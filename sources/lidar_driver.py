"""LightWare SF45/B scanning lidar -> timesync bus.

Everything below was measured on the connected unit (SF45, S/N S45-17584,
firmware 1.3.0, hardware 1, UART at 115200 through an FT232R adapter):

  Protocol     LWNX binary: 0xAA, u16 flags (len << 6 | write), cmd id, data,
               CRC-16/XMODEM. The distance packet (cmd 44) holds one int16 per
               field enabled in cmd 27, in bit order.
  Fields       first return (cm), first strength, last return (cm),
               temperature (0.01 degC), yaw (0.01 deg). No return comes as
               -100 cm with strength 0; otherwise strength reads 100.
  Point rate   rate 3 -> 200/s, 4 -> 400/s, 5, 6 and 7 all cap at ~478/s.
               Overloading the 115200 link (all 9 fields at the saved rate 12)
               made the unit reset and lose every unsaved setting.
  Scan         +-45 deg set; the head overshoots to ~+-46.2. One sweep (one
               direction) takes ~0.267 s, ~500 deg/s mid-arc and slower at the
               ends, so angle is never derived from time. Yaw steps are 0.45 deg.
  Latency      a yaw value arrives on the host ~4.7 ms after the head was at
               that angle (measured by querying cmd 97 while streaming,
               IQR 4.5-5.0 ms).
  Angle lag    the distance is measured ~0.9 ms before the yaw sent with it
               (from sweep hysteresis: 0.84-1.14 ms at 400-478 points/s).
  First-return freeze
               in some streaming sessions the first return holds one value
               (e.g. 1.25 m at every angle) for the whole session while the
               last return keeps tracking the scene; normally first == last.
               It came and went between runs with identical commands; the
               frozen value can be nearer or farther than the real one.
               FirstReturnCheck rejects such points (and any first return
               farther than the last), dist_m then uses the last return and
               the point is flagged first_rejected.
  Direction    raw yaw is positive to the right (checked with an obstacle on
               the right). angle_deg is flipped to positive = left, and
               x_m / y_m are forward / left in metres.
  Packets      ids >= 100 are the internal laser module's replies, forwarded
               unasked (seen: 127, 130). They are ignored. Ids 12, 14, 16, 17
               and anything >= 100 are never sent: a read sweep through that
               range rebooted the unit.

For every point the driver publishes lidar.pt at the estimated measurement
time, with the angle the head had at that time:
    t_meas = t_arrival - latency - angle_lag
    angle  = yaw series interpolated at t_meas
Points are grouped into sweeps, split where the yaw reverses, and each
finished sweep is published as one lidar.sweep record. Every point also goes
to <out_dir>/lidar_points.csv. Device identity, settings, constants and run
statistics go to <out_dir>/lidar_info.json.

Settings are changed in RAM only (never saved to flash) and restored on stop.
If the unit resets mid-run it comes back with its saved settings (yaw output
off, stream off); the watchdog sees the silence and configures it again.
"""

import json
import math
import os
import queue
from collections import deque
import struct
import sys
import threading
import time

import serial

from bus import Record
from csvlog import CsvLog
from sources.base import Source

START = 0xAA
MAX_PAYLOAD = 128

CMD_PRODUCT = 0
CMD_HARDWARE = 1
CMD_FIRMWARE = 2
CMD_SERIAL = 3
CMD_DIST_OUTPUT = 27
CMD_STREAM = 30
CMD_DIST_CM = 44
CMD_UPDATE_RATE = 66
CMD_BAUD = 79
CMD_SCAN_SPEED = 85
CMD_SCAN_ON_STARTUP = 94
CMD_SCAN_ENABLE = 96
CMD_SCAN_LOW = 98
CMD_SCAN_HIGH = 99
UNSAFE_CMDS = {12, 14, 16, 17}

STREAM_OFF = 0
STREAM_DIST_CM = 5

F_FIRST, F_FIRST_STRENGTH, F_LAST, F_TEMP, F_YAW = 0, 2, 3, 7, 8
FIELD_BITS = (F_FIRST, F_FIRST_STRENGTH, F_LAST, F_TEMP, F_YAW)
FIELD_NAMES = ["first_cm", "first_strength", "last_cm", "temp_centi_c",
               "yaw_centi_deg"]
FIELDS = sum(1 << b for b in FIELD_BITS)
PAYLOAD_LEN = 2 * len(FIELD_BITS)
PACKET_LEN = 3 + 1 + PAYLOAD_LEN + 2

RATE_POINTS_PER_S = {1: 50.0, 2: 100.0, 3: 200.0, 4: 400.0,
                     5: 478.0, 6: 478.0, 7: 478.0}

# The device's yaw is positive to the RIGHT: an obstacle placed to the right
# of the unit showed up at +20..+46 deg of raw yaw. angle_deg flips it so that
# positive is left (counter-clockwise from above, x forward, y left).
YAW_SIGN = -1

POINT_COLUMNS = ["t_ns", "t_unix", "t_arrival_ns", "sweep", "dir", "angle_deg",
                 "yaw_raw_deg", "dist_m", "x_m", "y_m", "first_m", "last_m",
                 "first_rejected", "strength", "temp_c", "batch"]

GAP_CLOSE_S = 0.2
WATCHDOG_S = 1.0

FIRST_STUCK_SAMPLES = 60
FIRST_STUCK_LAST_CHANGES = 10

MS = 1_000_000


def crc16(data):
    crc = 0
    for b in data:
        code = (crc >> 8) ^ b
        code ^= code >> 4
        crc = ((crc << 8) ^ code ^ (code << 5) ^ (code << 12)) & 0xFFFF
    return crc


def build_packet(cmd, data=b"", write=False):
    if cmd in UNSAFE_CMDS or not 0 <= cmd < 100:
        raise ValueError(f"refusing to send command {cmd}")
    body = struct.pack("<BHB", START, ((1 + len(data)) << 6) | int(write), cmd) + data
    return body + struct.pack("<H", crc16(body))


def sector_edges(low, high, n):
    return [low + i * (high - low) / n for i in range(n + 1)]


class Parser:

    def __init__(self):
        self._buf = bytearray()
        self.crc_errors = 0

    def reset(self):
        self._buf.clear()

    def feed(self, chunk):
        buf = self._buf
        buf.extend(chunk)
        out = []
        while True:
            i = buf.find(START)
            if i < 0:
                buf.clear()
                break
            if i:
                del buf[:i]
            if len(buf) < 3:
                break
            n = (buf[1] | buf[2] << 8) >> 6
            if n < 1 or n > MAX_PAYLOAD:
                del buf[0]
                continue
            total = 3 + n + 2
            if len(buf) < total:
                break
            body = bytes(buf[:3 + n])
            if crc16(body) != (buf[total - 2] | buf[total - 1] << 8):
                self.crc_errors += 1
                del buf[0]
                continue
            out.append((body[3], body[4:]))
            del buf[:total]
        return out


class FirstReturnCheck:
    """Decides, per point, whether the first return can be trusted.

    Rejected when it is farther than the last return (impossible for a real
    first return), or when it belongs to a run of `window` identical first
    returns during which the last return changed at least `min_last_changes`
    times (the freeze described above). Points are held until `window - 1`
    later points have arrived, so every point of a frozen run is caught,
    including the ones at its start. This delays points by ~125 ms.
    """

    def __init__(self, window=FIRST_STUCK_SAMPLES,
                 min_last_changes=FIRST_STUCK_LAST_CHANGES):
        self.window = window
        self.min_last_changes = min_last_changes
        self._q = deque()
        self._run = None

    def add(self, pt):
        first, last = pt["first_cm"], pt["last_cm"]
        r = self._run
        if r is None or first != r["value"]:
            r = self._run = {"value": first, "n": 0, "changes": 0,
                             "last": last, "frozen": False}
        else:
            r["changes"] += last != r["last"]
            r["last"] = last
        r["n"] += 1
        if r["n"] >= self.window and r["changes"] >= self.min_last_changes:
            r["frozen"] = True
        pt["_run"] = r
        self._q.append(pt)
        out = []
        while len(self._q) >= self.window:
            out.append(self._decide(self._q.popleft()))
        return out

    def flush(self):
        out = [self._decide(p) for p in self._q]
        self._q.clear()
        self._run = None
        return out

    @staticmethod
    def _decide(pt):
        r = pt.pop("_run")
        behind = (pt["first"] is not None and pt["last"] is not None
                  and pt["first"] > pt["last"])
        pt["first_rejected"] = int(r["frozen"] or behind)
        return pt


class SweepSplitter:
    """Splits the point stream into sweeps where the yaw reverses.

    A reversal is accepted once the yaw has come back `reverse_deg` from the
    furthest angle reached. The furthest point closes the old sweep and the
    points after it open the new one. Points after the current extreme are
    held back until it is clear which sweep they belong to (a few samples at
    the ends of the arc), so every point is released with its final sweep id.
    """

    def __init__(self, reverse_deg=0.9, max_points=20000):
        self.reverse_deg = reverse_deg
        self.max_points = max_points
        self.sweep_id = 0
        self.dir = 0
        self._ext = None
        self._tail = []
        self._points = []

    def add(self, pt):
        """Returns (released_points, finished_sweep or None).
        A finished sweep is (sweep_id, dir, points)."""
        y = pt["yaw"]
        released, finished = [], None
        if self.dir == 0:
            if self._ext is None:
                self._ext = y
            elif abs(y - self._ext) >= self.reverse_deg:
                self.dir = 1 if y > self._ext else -1
                self._ext = y
            released.append(pt)
        elif (y - self._ext) * self.dir > 0:
            self._ext = y
            released.extend(self._tail)
            released.append(pt)
            self._tail = []
        elif (self._ext - y) * self.dir >= self.reverse_deg:
            finished = (self.sweep_id, self.dir, self._points)
            self.sweep_id += 1
            self.dir = -self.dir
            self._points = []
            rest = self._tail + [pt]
            k = max(range(len(rest)), key=lambda j: rest[j]["yaw"] * self.dir)
            self._ext = rest[k]["yaw"]
            released.extend(rest[:k + 1])
            self._tail = rest[k + 1:]
        else:
            self._tail.append(pt)
            if len(self._tail) > 200:
                released.extend(self._tail)
                self._tail = []
        self._assign(released)
        if finished is None and len(self._points) >= self.max_points:
            more, finished = self.flush()
            released.extend(more)
        return released, finished

    def pending(self):
        return bool(self._points or self._tail)

    def flush(self):
        """Ends the current sweep now (stream gap or stop).
        Returns (released_points, finished_sweep or None)."""
        released = self._tail
        self._assign(released)
        pts = self._points
        finished = (self.sweep_id, self.dir, pts) if pts else None
        if pts:
            self.sweep_id += 1
        self._points, self._tail = [], []
        self._ext = None
        self.dir = 0
        return released, finished

    def _assign(self, pts):
        for p in pts:
            p["sweep"] = self.sweep_id
            p["dir"] = self.dir
        self._points.extend(pts)


class LidarSource(Source):

    def __init__(self, bus, clock, port, baud=115200, low_angle=None,
                 high_angle=None, n_sectors=12, update_rate=6, out_dir=None,
                 latency_ms=4.7, angle_lag_ms=0.9, name="lidar"):
        super().__init__(name, bus, clock)
        for label, v, lo, hi in (("low_angle", low_angle, -160, -10),
                                 ("high_angle", high_angle, 10, 160)):
            if v is not None and not lo <= v <= hi:
                raise ValueError(f"{label} must be in [{lo}, {hi}], got {v}")
        if update_rate not in RATE_POINTS_PER_S:
            raise ValueError(f"update_rate must be one of "
                             f"{sorted(RATE_POINTS_PER_S)} (8-12 are untested "
                             f"on this unit and give no more than 7), "
                             f"got {update_rate}")
        bits_per_s = PACKET_LEN * 10 * RATE_POINTS_PER_S[update_rate]
        if bits_per_s > 0.8 * baud:
            raise ValueError(f"update rate {update_rate} needs {bits_per_s:.0f} "
                             f"bit/s, more than 80% of {baud} baud")
        if n_sectors < 1:
            raise ValueError("n_sectors must be at least 1")
        self.port = port
        self.baud = baud
        self.low_angle = low_angle
        self.high_angle = high_angle
        self.n_sectors = n_sectors
        self.update_rate = update_rate
        self.out_dir = out_dir
        self.latency_ns = int(latency_ms * MS)
        self.angle_lag_ns = int(angle_lag_ms * MS)

        self._ser = None
        self._parser = Parser()
        self._splitter = SweepSplitter()
        self._stash = []
        self._seq = 0
        self._prev_yaw = None
        self._last_arrival = None
        self._period_ns = 1e9 / RATE_POINTS_PER_S[update_rate]
        self._last_point_mono = time.monotonic()
        self._next_retry = 0.0
        self._originals = None
        self.device = {}
        self.scan_low = None
        self.scan_high = None
        self._edges = None

        self.points = 0
        self.invalid = 0
        self.sweeps = 0
        self.batched_reads = 0
        self.bad_payload = 0
        self.other_packets = 0
        self.reconfigures = 0
        self.reopens = 0
        self.first_rejected = 0
        self._t_first = None
        self._t_last = None
        self._check = FirstReturnCheck()

        self._rows = queue.Queue()
        self._writer = None
        self._points_log = None

    # ---------------------------------------------------------------- serial

    def _open(self):
        ser = serial.Serial(self.port, self.baud, timeout=0.02,
                            write_timeout=0.5, exclusive=True)
        try:
            ser.set_low_latency_mode(True)
        except (AttributeError, NotImplementedError, OSError, ValueError,
                serial.SerialException):
            pass
        self._ser = ser

    def _close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def _read(self):
        b = self._ser.read(self._ser.in_waiting or 1)
        return b, self.clock.mono_ns()

    def _transact(self, cmd, data=b"", write=False, timeout=0.25, retries=4):
        pkt = build_packet(cmd, data, write)
        for _ in range(retries):
            self._ser.write(pkt)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                b, t = self._read()
                if not b:
                    continue
                for c, d in self._parser.feed(b):
                    if c == cmd:
                        return d
                    if c == CMD_DIST_CM:
                        self._stash.append((t, d))
        raise IOError(f"SF45/B: no reply to command {cmd}")

    def _read_fmt(self, cmd, fmt):
        return struct.unpack_from(fmt, self._transact(cmd))[0]

    def _write_fmt(self, cmd, fmt, value, check=True):
        reply = self._transact(cmd, struct.pack(fmt, value), write=True)
        if check:
            got = struct.unpack_from(fmt, reply)[0]
            bad = abs(got - value) > 0.01 if isinstance(value, float) else got != value
            if bad:
                raise IOError(f"SF45/B: wrote {value} to command {cmd}, "
                              f"device reports {got}")

    def _text(self, cmd):
        return self._transact(cmd).split(b"\0", 1)[0].decode(errors="replace")

    # ------------------------------------------------------------- configure

    def _configure(self):
        self._write_fmt(CMD_STREAM, "<I", STREAM_OFF, check=False)
        time.sleep(0.05)
        self._ser.reset_input_buffer()
        self._parser.reset()
        self._stash.clear()

        product = self._text(CMD_PRODUCT)
        if not product.startswith("SF45"):
            raise IOError(f"{self.port} reports {product!r}, not an SF45/B")
        fw = self._read_fmt(CMD_FIRMWARE, "<I")
        self.device.update({
            "product": product,
            "serial": self._text(CMD_SERIAL),
            "hardware": self._read_fmt(CMD_HARDWARE, "<I"),
            "firmware": f"{(fw >> 16) & 0xFF}.{(fw >> 8) & 0xFF}.{fw & 0xFF}",
        })

        if self._originals is None:
            self._originals = {
                "fields": self._read_fmt(CMD_DIST_OUTPUT, "<I"),
                "update_rate": self._read_fmt(CMD_UPDATE_RATE, "<B"),
                "scan_low": self._read_fmt(CMD_SCAN_LOW, "<f"),
                "scan_high": self._read_fmt(CMD_SCAN_HIGH, "<f"),
            }

        self._write_fmt(CMD_DIST_OUTPUT, "<I", FIELDS)
        self._write_fmt(CMD_UPDATE_RATE, "<B", self.update_rate)
        if self.low_angle is not None:
            self._write_fmt(CMD_SCAN_LOW, "<f", float(self.low_angle))
        if self.high_angle is not None:
            self._write_fmt(CMD_SCAN_HIGH, "<f", float(self.high_angle))
        self.scan_low = round(self._read_fmt(CMD_SCAN_LOW, "<f"), 3)
        self.scan_high = round(self._read_fmt(CMD_SCAN_HIGH, "<f"), 3)
        self._edges = sector_edges(*self._scan_range(), self.n_sectors)
        self.device.update({
            "scan_speed": self._read_fmt(CMD_SCAN_SPEED, "<H"),
            "scan_enabled": self._read_fmt(CMD_SCAN_ENABLE, "<B"),
            "scan_on_startup": self._read_fmt(CMD_SCAN_ON_STARTUP, "<B"),
            "baud_setting": self._read_fmt(CMD_BAUD, "<B"),
        })
        if not self.device["scan_enabled"]:
            print(f"[{self.name}] warning: scanning is disabled on the device, "
                  f"the head will not sweep", file=sys.stderr, flush=True)

        self._prev_yaw = None
        self._last_arrival = None
        self._write_fmt(CMD_STREAM, "<I", STREAM_DIST_CM)
        self._last_point_mono = time.monotonic()

    def _scan_range(self):
        """Scan limits in the output frame (angle_deg), low to high."""
        a, b = YAW_SIGN * self.scan_low, YAW_SIGN * self.scan_high
        return min(a, b), max(a, b)

    def _restore(self):
        if self._ser is None or self._originals is None:
            return
        o = self._originals
        try:
            self._write_fmt(CMD_STREAM, "<I", STREAM_OFF, check=False)
            self._write_fmt(CMD_DIST_OUTPUT, "<I", o["fields"], check=False)
            self._write_fmt(CMD_UPDATE_RATE, "<B", o["update_rate"], check=False)
            if self.low_angle is not None:
                self._write_fmt(CMD_SCAN_LOW, "<f", o["scan_low"], check=False)
            if self.high_angle is not None:
                self._write_fmt(CMD_SCAN_HIGH, "<f", o["scan_high"], check=False)
        except Exception as e:
            print(f"[{self.name}] could not restore device settings: {e}",
                  file=sys.stderr, flush=True)

    # ------------------------------------------------------------- lifecycle

    def setup(self):
        self._open()
        try:
            self._configure()
        except Exception:
            self._close()
            raise
        if self.out_dir:
            os.makedirs(self.out_dir, exist_ok=True)
            self._points_log = CsvLog(os.path.join(self.out_dir, "lidar_points.csv"),
                                      POINT_COLUMNS)
            self._writer = threading.Thread(target=self._write_rows,
                                            name=f"{self.name}-csv", daemon=True)
            self._writer.start()
            self._write_info()

    def poll(self):
        if self._ser is None:
            self._reconnect()
            return
        try:
            b, t = self._read()
        except (serial.SerialException, OSError) as e:
            print(f"[{self.name}] serial error: {e}; reconnecting",
                  file=sys.stderr, flush=True)
            self._end_sweep()
            self._close()
            return
        if self._stash:
            stash, self._stash = self._stash, []
            for ts, d in stash:
                self._handle(d, ts, 1)
        if b:
            pkts = []
            for c, d in self._parser.feed(b):
                if c == CMD_DIST_CM:
                    pkts.append(d)
                else:
                    self.other_packets += 1
            self._handle_batch(pkts, t)
        now = time.monotonic()
        silent = now - self._last_point_mono
        if silent > GAP_CLOSE_S and self._splitter.pending():
            self._end_sweep()
        if silent > WATCHDOG_S and now >= self._next_retry:
            self._watchdog()

    def teardown(self):
        self._end_sweep()
        self._restore()
        self._close()
        if self._writer is not None:
            self._rows.put(None)
            self._writer.join(timeout=5.0)
        if self._points_log is not None:
            self._points_log.close()
        if self.out_dir:
            self._write_info()

    # ---------------------------------------------------------------- points

    def _handle_batch(self, pkts, t_read):
        """All packets completed by one read share its timestamp. When the
        adapter delivers several at once they were sent one period apart, so
        they are spread back from the read time."""
        k = len(pkts)
        if k == 0:
            return
        if k > 1:
            self.batched_reads += 1
        elif self._last_arrival is not None:
            dt = t_read - self._last_arrival
            if 0.5 * self._period_ns < dt < 2.0 * self._period_ns:
                self._period_ns += (dt - self._period_ns) / 100.0
        for j, d in enumerate(pkts):
            t_arr = t_read - int((k - 1 - j) * self._period_ns)
            if self._last_arrival is not None and t_arr <= self._last_arrival:
                t_arr = self._last_arrival + 1
            self._handle(d, t_arr, k)

    def _handle(self, d, t_arr, batch):
        if len(d) != PAYLOAD_LEN:
            self.bad_payload += 1
            return
        self._last_arrival = t_arr
        first, strength, last, temp, yaw = struct.unpack("<5h", d)
        yaw_deg = yaw / 100.0
        t_yaw = t_arr - self.latency_ns
        t_meas = t_yaw - self.angle_lag_ns

        angle = yaw_deg
        prev = self._prev_yaw
        if prev is not None and 0 < t_yaw - prev[0] <= 4 * self._period_ns:
            w = (t_meas - prev[0]) / (t_yaw - prev[0])
            w = min(1.0, max(0.0, w))
            angle = prev[1] + w * (yaw_deg - prev[1])
        self._prev_yaw = (t_yaw, yaw_deg)

        pt = {"t": t_meas, "t_arr": t_arr, "yaw": yaw_deg,
              "angle": round(YAW_SIGN * angle, 3),
              "first_cm": first, "last_cm": last,
              "first": first / 100.0 if first > 0 else None,
              "last": last / 100.0 if last > 0 else None,
              "strength": strength, "temp": temp / 100.0, "batch": batch}
        self.points += 1
        if self._t_first is None:
            self._t_first = t_arr
        self._t_last = t_arr
        self._last_point_mono = time.monotonic()
        for p in self._check.add(pt):
            self._accept(p)

    def _accept(self, p):
        """Second stage, once the first-return check has decided."""
        dist = p["last"] if p["first_rejected"] else p["first"]
        p["dist"] = dist
        p["x"] = p["y"] = None
        if dist is not None:
            a = math.radians(p["angle"])
            p["x"] = round(dist * math.cos(a), 4)
            p["y"] = round(dist * math.sin(a), 4)
        self.first_rejected += p["first_rejected"]
        if dist is None:
            self.invalid += 1
        released, finished = self._splitter.add(p)
        self._release(released, finished)

    def _end_sweep(self):
        for p in self._check.flush():
            self._accept(p)
        released, finished = self._splitter.flush()
        self._release(released, finished)
        self._prev_yaw = None

    def _release(self, released, finished):
        now = self.clock.mono_ns()
        for p in released:
            self._seq += 1
            # t_arrival_ns is when the point was published (after the ~125 ms
            # first-return hold); the packet's own arrival is in the CSV.
            self.bus.publish(Record(
                "lidar.pt", t_host_ns=p["t"], t_arrival_ns=now,
                fields={"angle_deg": p["angle"], "dist_m": p["dist"],
                        "x_m": p["x"], "y_m": p["y"], "sweep": p["sweep"]},
                seq=self._seq))
            if self._points_log is not None:
                self._rows.put(p)
        if finished is not None:
            self._publish_sweep(*finished)

    def _publish_sweep(self, sweep_id, direction, pts):
        (lo, hi), n = self._scan_range(), self.n_sectors
        width = (hi - lo) / n
        sectors = [None] * n
        valid = []
        for p in pts:
            d = p["dist"]
            if d is None:
                continue
            valid.append(d)
            i = min(n - 1, max(0, int((p["angle"] - lo) // width)))
            if sectors[i] is None or d < sectors[i]:
                sectors[i] = d
        angles = [p["angle"] for p in pts]
        fields = {
            "id": sweep_id,
            "n_points": len(pts),
            "n_valid": len(valid),
            "sweep_ms": round((pts[-1]["t"] - pts[0]["t"]) / 1e6, 1),
            "dir": YAW_SIGN * direction,
            "angle_lo": min(angles),
            "angle_hi": max(angles),
            "range_min": min(valid) if valid else None,
            "temp_c": round(sum(p["temp"] for p in pts) / len(pts), 2),
        }
        for i, v in enumerate(sectors):
            fields[f"s{i:02d}"] = v
        self.sweeps += 1
        self._seq += 1
        # t_arrival_ns is where this sweep's data ends. The aligner treats the
        # stream as covered up to it; the packet arrival time would be too
        # late, because the next sweep starts ~2 ms after this one's last point.
        self.bus.publish(Record("lidar.sweep", t_host_ns=pts[0]["t"],
                                t_arrival_ns=pts[-1]["t"], fields=fields,
                                seq=self._seq))

    # ---------------------------------------------------------------- output

    def _write_rows(self):
        to_unix = self.clock.to_unix_ns
        done = False
        while not done:
            batch = [self._rows.get()]
            while len(batch) < 1000:
                try:
                    batch.append(self._rows.get_nowait())
                except queue.Empty:
                    break
            rows = []
            for p in batch:
                if p is None:
                    done = True
                    break
                rows.append({
                    "t_ns": p["t"], "t_unix": f"{to_unix(p['t']) / 1e9:.6f}",
                    "t_arrival_ns": p["t_arr"], "sweep": p["sweep"],
                    "dir": YAW_SIGN * p["dir"], "angle_deg": p["angle"],
                    "yaw_raw_deg": p["yaw"], "dist_m": p["dist"],
                    "x_m": p["x"], "y_m": p["y"],
                    "first_m": p["first"], "last_m": p["last"],
                    "first_rejected": p["first_rejected"], "strength": p["strength"],
                    "temp_c": p["temp"], "batch": p["batch"]})
            try:
                self._points_log.write_many(rows)
            except Exception as e:
                print(f"[{self.name}] lidar_points.csv write failed: {e}",
                      file=sys.stderr, flush=True)

    def measured_rate(self):
        if self._t_first is None or self._t_last == self._t_first:
            return None
        return (self.points - 1) * 1e9 / (self._t_last - self._t_first)

    def _write_info(self):
        rate = self.measured_rate()
        info = {
            "device": self.device,
            "port": self.port,
            "baud": self.baud,
            "fields_mask": FIELDS,
            "fields": FIELD_NAMES,
            "update_rate_setting": self.update_rate,
            "points_per_s_expected": RATE_POINTS_PER_S[self.update_rate],
            "points_per_s_measured": None if rate is None else round(rate, 2),
            "angle_convention": "angle_deg: 0 = straight ahead, positive = "
                                "left (counter-clockwise seen from above); "
                                "x_m forward, y_m left; yaw_raw_deg = device "
                                "yaw, angle_deg = yaw_sign * yaw",
            "yaw_sign": YAW_SIGN,
            "scan_low_deg": self.scan_low,
            "scan_high_deg": self.scan_high,
            "sector_edges_deg": self._edges,
            "latency_ms": self.latency_ns / MS,
            "angle_lag_ms": self.angle_lag_ns / MS,
            "timestamp_model": "t_ns = t_arrival_ns - latency_ms - angle_lag_ms; "
                               "angle_deg = yaw interpolated at t_ns",
            "device_settings_before_run": self._originals,
            "stats": {
                "points": self.points, "invalid": self.invalid,
                "first_rejected": self.first_rejected,
                "sweeps": self.sweeps, "batched_reads": self.batched_reads,
                "crc_errors": self._parser.crc_errors,
                "bad_payload": self.bad_payload,
                "other_packets": self.other_packets,
                "reconfigures": self.reconfigures, "reopens": self.reopens,
                "source_errors": self.errors,
            },
        }
        path = os.path.join(self.out_dir, "lidar_info.json")
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(info, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)

    # -------------------------------------------------------------- recovery

    def _watchdog(self):
        self._next_retry = time.monotonic() + 2.0
        self._end_sweep()
        print(f"[{self.name}] no data for 1 s, configuring the device again",
              file=sys.stderr, flush=True)
        try:
            self._configure()
            self.reconfigures += 1
        except (IOError, OSError, serial.SerialException) as e:
            print(f"[{self.name}] reconfigure failed: {e}", file=sys.stderr,
                  flush=True)

    def _reconnect(self):
        if time.monotonic() < self._next_retry:
            self._stop_evt.wait(0.05)
            return
        self._next_retry = time.monotonic() + 1.0
        try:
            self._open()
            self._configure()
            self.reopens += 1
            print(f"[{self.name}] reconnected", file=sys.stderr, flush=True)
        except Exception as e:
            self._close()
            print(f"[{self.name}] reconnect failed: {e}", file=sys.stderr,
                  flush=True)

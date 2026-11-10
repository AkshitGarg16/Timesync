"""MAVLink autopilot (via MAVProxy or direct) -> timesync bus.

Intended wiring: MAVProxy owns the autopilot link and forwards to a UDP port
this source listens on, so a GCS can share the link:

    mavproxy.py --master=/dev/serial/by-id/usb-CubePilot_CubeOrange+_...-if00 \\
                --baudrate 921600 --out=udp:127.0.0.1:14550 --streamrate=-1
    python3 main.py --fcu udpin:0.0.0.0:14550 ...

When the messages later arrive over the network, point MAVProxy's --out (or
the aircraft's router) at this host and keep --fcu udpin:0.0.0.0:14550. A
direct serial port (--fcu /dev/ttyACM0 --baud 921600) also works, but then
nothing else can use that port. --streamrate=-1 stops MAVProxy from resetting
stream rates; this source sets its own with SET_MESSAGE_INTERVAL.

Measured on the connected unit (CubeOrange+, ArduCopter, USB, MAVProxy 1.8.74
relaying to UDP localhost):

  Rates        ATTITUDE at 50 Hz and GLOBAL_POSITION_INT at 10 Hz accepted
               through MAVProxy and delivered at the requested rate.
  Timesync     TIMESYNC round trip host -> MAVProxy -> FC -> MAVProxy -> host
               2.7-8.3 ms, median 4.8 ms (n=136). The FC answers with tc1 in
               boot-time ns (micros64 * 1000), the clock time_boot_ms uses.
  Stamps       ArduPilot fills time_boot_ms with millis() when the message is
               sent, so it is the send time truncated to 1 ms. Half a
               millisecond is added to make it unbiased.

Clock mapping: TIMESYNC requests go out at `timesync_hz` (faster until the
filter converges) and each reply feeds ClockOffset.observe_roundtrip. If no
reply ever comes (a receive-only UDP link), the source falls back to one-way
offsets from message arrival, which include the link latency. Each message is
published at its time_boot_ms mapped to host monotonic time, never later
than its arrival. Before the filter converges messages are stamped with
arrival time (ClockOffset.to_host).

Publishes fcu.att (rad, rad/s) and fcu.gpos (deg, m, m/s, heading deg or
None). With out_dir set, every message goes to fcu_att.csv / fcu_gpos.csv
with its raw device time, every timesync round trip to fcu_timesync.csv (so
the mapping can be refit offline), and link info and stats to fcu_info.json.
"""

import json
import os
import sys
import time

from pymavlink import mavutil

from bus import Record
from clock import ClockOffset, Uint32Unwrapper
from csvlog import CsvLog
from sources.base import Source

MS = 1_000_000

MSG_ATTITUDE = 30
MSG_GLOBAL_POSITION_INT = 33
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_COMP_ID_ONBOARD_COMPUTER = 191

HALF_TICK_NS = MS // 2
REBOOT_JUMP_MS = 5000
TIMESYNC_WARMUP_HZ = 50.0
TIMESYNC_ONEWAY_AFTER_S = 5.0
RATE_CHECK_S = 3.0

ATT_COLUMNS = ["roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"]
GPOS_COLUMNS = ["lat", "lon", "alt", "relative_alt", "vx", "vy", "vz", "hdg"]
STAMP_COLUMNS = ["t_ns", "t_unix", "t_arrival_ns", "t_device_ns", "time_boot_ms"]
SYNC_COLUMNS = ["t_sent_ns", "t_device_ns", "t_recv_ns", "rtt_ms", "accepted",
                "offset_ns", "skew_ns", "converged"]


def att_fields(m):
    return {"roll": m.roll, "pitch": m.pitch, "yaw": m.yaw,
            "rollspeed": m.rollspeed, "pitchspeed": m.pitchspeed,
            "yawspeed": m.yawspeed}


def gpos_fields(m):
    return {"lat": m.lat / 1e7, "lon": m.lon / 1e7, "alt": m.alt / 1000.0,
            "relative_alt": m.relative_alt / 1000.0,
            "vx": m.vx / 100.0, "vy": m.vy / 100.0, "vz": m.vz / 100.0,
            "hdg": None if m.hdg == 65535 else m.hdg / 100.0}


class MavlinkSource(Source):

    def __init__(self, bus, clock, conn, baud=921600, out_dir=None,
                 att_hz=50.0, gpos_hz=10.0, timesync_hz=10.0, max_rtt_ms=10.0,
                 request_rates=True, name="fcu"):
        super().__init__(name, bus, clock)
        self.conn = conn
        self.baud = baud
        self.out_dir = out_dir
        self.request_rates = request_rates
        self.timesync_period = 1.0 / timesync_hz
        self.streams = {
            "ATTITUDE": ("fcu.att", MSG_ATTITUDE, att_hz, att_fields),
            "GLOBAL_POSITION_INT": ("fcu.gpos", MSG_GLOBAL_POSITION_INT,
                                    gpos_hz, gpos_fields),
        }

        self.offset = ClockOffset(max_rtt_ns=int(max_rtt_ms * MS))
        self._unwrap = Uint32Unwrapper()
        self._last_boot_ms = None
        self._m = None
        self.target_system = None
        self.autopilot = None
        self.vehicle_type = None
        self._pending = {}
        self._next_sync = 0.0
        self._first_sync_sent = None
        self.sync_mode = None
        self._window = {}
        self._window_start = None
        self._next_rate_request = 0.0

        self.counts = {}
        self.clamped = 0
        self.reboots = 0
        self.rate_requests = 0
        self.sync_sent = 0
        self.sync_replies = 0
        self.foreign = 0
        self._seq = 0
        self._logs = {}

    # ------------------------------------------------------------- lifecycle

    def setup(self):
        self._m = mavutil.mavlink_connection(
            self.conn, baud=self.baud, source_system=255,
            source_component=MAV_COMP_ID_ONBOARD_COMPUTER, autoreconnect=True)
        if self.out_dir:
            os.makedirs(self.out_dir, exist_ok=True)
            for msg_type, (stream, _, _, _) in self.streams.items():
                cols = ATT_COLUMNS if stream == "fcu.att" else GPOS_COLUMNS
                path = os.path.join(self.out_dir,
                                    stream.replace(".", "_") + ".csv")
                self._logs[msg_type] = CsvLog(path, STAMP_COLUMNS + cols)
            self._logs["TIMESYNC"] = CsvLog(
                os.path.join(self.out_dir, "fcu_timesync.csv"), SYNC_COLUMNS)
            self._write_info()

    def poll(self):
        now = time.monotonic()
        if self.target_system is not None:
            if now >= self._next_sync:
                self._send_timesync(now)
            self._check_rates(now)
        wait = max(0.0, min(0.02, self._next_sync - now))
        msg = self._m.recv_match(blocking=True, timeout=wait)
        if msg is None:
            return
        t_arr = self.clock.mono_ns()
        self._handle(msg, t_arr)
        while (msg := self._m.recv_match(blocking=False)) is not None:
            self._handle(msg, self.clock.mono_ns())

    def teardown(self):
        if self._m is not None:
            try:
                self._m.close()
            except Exception:
                pass
        for log in self._logs.values():
            log.close()
        if self.out_dir:
            self._write_info()

    # -------------------------------------------------------------- messages

    def _handle(self, msg, t_arr):
        t = msg.get_type()
        if t == "BAD_DATA":
            return
        src_sys, src_comp = msg.get_srcSystem(), msg.get_srcComponent()
        if t == "HEARTBEAT" and self.target_system is None:
            if (src_comp == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                    and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                self._on_autopilot(msg, src_sys)
            return
        if (src_sys != self.target_system
                or src_comp != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
            self.foreign += 1
            return
        if t == "TIMESYNC":
            self._on_timesync(msg, t_arr)
            return
        spec = self.streams.get(t)
        if spec is not None:
            self._on_stream(msg, t, spec, t_arr)

    def _on_autopilot(self, hb, sysid):
        self.target_system = sysid
        self.autopilot = mavutil.mavlink.enums["MAV_AUTOPILOT"][hb.autopilot].name
        self.vehicle_type = mavutil.mavlink.enums["MAV_TYPE"][hb.type].name
        print(f"[{self.name}] autopilot sysid {sysid} ({self.autopilot}, "
              f"{self.vehicle_type}) on {self.conn}", file=sys.stderr, flush=True)
        self._request_rates()
        self._window_start = time.monotonic()

    def _on_stream(self, msg, msg_type, spec, t_arr):
        stream, _, _, to_fields = spec
        boot_ms = msg.time_boot_ms
        if (self._last_boot_ms is not None
                and boot_ms + REBOOT_JUMP_MS < self._last_boot_ms
                and self._last_boot_ms - boot_ms < (1 << 31)):
            self._on_reboot(boot_ms)
        self._last_boot_ms = boot_ms
        t_dev = self._unwrap.to_ns(boot_ms) + HALF_TICK_NS

        if self.sync_mode == "oneway" and msg_type == "ATTITUDE":
            self.offset.observe_oneway(t_dev, t_arr)
        t_host = self.offset.to_host(t_dev, t_arr)
        if t_host > t_arr:
            self.clamped += 1
            t_host = t_arr

        fields = to_fields(msg)
        self._seq += 1
        self.bus.publish(Record(stream, t_host, t_arr, fields,
                                t_device_ns=t_dev, seq=self._seq))
        self.counts[stream] = self.counts.get(stream, 0) + 1
        self._window[msg_type] = self._window.get(msg_type, 0) + 1
        log = self._logs.get(msg_type)
        if log is not None:
            log.write({"t_ns": t_host,
                       "t_unix": f"{self.clock.to_unix_ns(t_host) / 1e9:.6f}",
                       "t_arrival_ns": t_arr, "t_device_ns": t_dev,
                       "time_boot_ms": boot_ms, **fields})

    def _on_reboot(self, boot_ms):
        self.reboots += 1
        print(f"[{self.name}] autopilot time_boot_ms jumped back "
              f"{self._last_boot_ms} -> {boot_ms}: reboot, resetting clock sync",
              file=sys.stderr, flush=True)
        self._unwrap = Uint32Unwrapper()
        self.offset.reset()
        self._pending.clear()
        self._next_rate_request = 0.0

    # -------------------------------------------------------------- timesync

    def _send_timesync(self, now):
        ts = self.clock.mono_ns()
        self._pending[ts] = ts
        if len(self._pending) > 64:
            for k in sorted(self._pending)[:-64]:
                del self._pending[k]
        self._m.mav.timesync_send(0, ts)
        self.sync_sent += 1
        if self._first_sync_sent is None:
            self._first_sync_sent = now
        if (self.sync_mode is None and self.sync_replies == 0
                and now - self._first_sync_sent > TIMESYNC_ONEWAY_AFTER_S):
            self.sync_mode = "oneway"
            self.offset.reset()
            print(f"[{self.name}] no TIMESYNC replies in "
                  f"{TIMESYNC_ONEWAY_AFTER_S:.0f} s; using one-way offsets "
                  f"(include link latency)", file=sys.stderr, flush=True)
        period = self.timesync_period
        if self.sync_mode != "oneway" and not self.offset.converged:
            period = min(period, 1.0 / TIMESYNC_WARMUP_HZ)
        elif self.sync_mode == "oneway":
            period = max(period, 1.0)
        self._next_sync = now + period

    def _on_timesync(self, msg, t_arr):
        if msg.tc1 == 0:
            return
        sent = self._pending.pop(msg.ts1, None)
        if sent is None:
            return
        self.sync_replies += 1
        if self.sync_mode != "roundtrip":
            if self.sync_mode == "oneway":
                self.offset.reset()
            self.sync_mode = "roundtrip"
        ok = self.offset.observe_roundtrip(sent, msg.tc1, t_arr)
        log = self._logs.get("TIMESYNC")
        if log is not None:
            o = self.offset
            log.write({"t_sent_ns": sent, "t_device_ns": msg.tc1,
                       "t_recv_ns": t_arr, "rtt_ms": round(o.last_rtt_ns / MS, 3),
                       "accepted": int(ok), "offset_ns": int(o.offset_ns),
                       "skew_ns": round(o.skew_ns, 3),
                       "converged": int(o.converged)})

    # ----------------------------------------------------------------- rates

    def _request_rates(self):
        if not self.request_rates:
            return
        for _, msg_id, hz, _ in self.streams.values():
            self._m.mav.command_long_send(
                self.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, int(1e6 / hz),
                0, 0, 0, 0, 0)
        self.rate_requests += 1
        self._next_rate_request = time.monotonic() + 5.0

    def _check_rates(self, now):
        """Re-request if a stream falls below half its rate, e.g. after an
        autopilot reboot or a GCS changing stream rates."""
        if self._window_start is None or now - self._window_start < RATE_CHECK_S:
            return
        span = now - self._window_start
        slow = [t for t, (_, _, hz, _) in self.streams.items()
                if self._window.get(t, 0) < 0.5 * hz * span]
        self._window.clear()
        self._window_start = now
        if slow and now >= self._next_rate_request:
            print(f"[{self.name}] {', '.join(slow)} below half rate; "
                  f"requesting again", file=sys.stderr, flush=True)
            self._request_rates()

    # ------------------------------------------------------------------ info

    def _write_info(self):
        o = self.offset
        info = {
            "conn": self.conn,
            "baud": self.baud,
            "target_system": self.target_system,
            "autopilot": self.autopilot,
            "vehicle_type": self.vehicle_type,
            "requested_hz": {t: hz for t, (_, _, hz, _) in self.streams.items()},
            "sync_mode": self.sync_mode,
            "offset_ns": int(o.offset_ns),
            "skew_ns": o.skew_ns,
            "converged": o.converged,
            "sync_sent": self.sync_sent,
            "sync_replies": self.sync_replies,
            "sync_accepted": o.accepted,
            "sync_rejected": o.rejected,
            "max_rtt_ns": o.max_rtt_ns,
            "counts": self.counts,
            "clamped_to_arrival": self.clamped,
            "reboots": self.reboots,
            "rate_requests": self.rate_requests,
            "foreign_messages": self.foreign,
            "errors": self.errors,
        }
        with open(os.path.join(self.out_dir, "fcu_info.json"), "w") as f:
            json.dump(info, f, indent=2)

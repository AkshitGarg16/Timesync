"""ROS-free, pure-Python driver for the NeoHD PTZ camera.

Three pieces, all built on `visca.py`:

* `NeoHDCamera`  - local object: opens (and flocks) the VISCA serial port,
  grabs video with OpenCV, and exposes every control as a method.
* `CameraServer` - streams the camera's video as JPEGs over TCP and accepts
  control commands from remote clients (started via `NeoHDCamera.serve()`).
* `NeoHDClient`  - remote object with the same control methods plus
  `read()`/`frames()` for the network video feed.

Local use:

    cam = NeoHDCamera("/dev/ttyACM0", video_device=0)
    frame = cam.read()
    cam.zoom(+1)          # drive in; cam.zoom(0) stops
    cam.set_exposure_mode("manual")

Streaming:

    # on the machine with the camera
    cam.serve(host="0.0.0.0", port=5600)          # blocks; serve(block=False)

    # anywhere else
    cli = NeoHDClient("camera-host", 5600)
    frame = cli.read()
    cli.zoom(+1)

Wire format (both directions): 1-byte type, 4-byte big-endian length, payload.
Type 0 = JPEG frame (server->client), 1 = JSON command (client->server),
2 = JSON reply (server->client).

Like the ROS node, settings with no VISCA inquiry wired up reflect only what
was last commanded, and the serial device is locked exclusively.
"""
import fcntl
import json
import os
import queue
import socket
import struct
import sys
import threading
import time
from typing import Optional

import cv2
import numpy as np
import serial

from . import visca
from avi import MjpegAviWriter
from bus import Record
from sources.base import Source

MSG_FRAME, MSG_COMMAND, MSG_REPLY = 0, 1, 2
_HEADER = struct.Struct(">BI")
_MAX_MSG = 64 * 1024 * 1024
DEFAULT_PORT = 5600

# Methods a remote client may invoke. Anything not listed here is refused.
CONTROL_METHODS = (
    "zoom", "set_zoom_position", "reset_zoom", "get_zoom", "get_focus",
    "focus", "set_focus_auto", "set_focus_position", "focus_one_push",
    "set_image_stabilizer",
    "set_exposure_mode", "set_shutter", "set_iris", "set_gain",
    "set_exposure_comp",
    "set_wb_mode", "set_wb_red_gain", "set_wb_blue_gain",
    "set_icr", "set_auto_icr", "set_auto_icr_threshold",
    "set_wdr", "set_defog", "set_backlight_compensation", "set_hlc",
    "set_high_sensitivity", "stop",
)


class NeoHDCamera:
    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        video_device=0,
        baud: int = 9600,
        drive_speed: int = 4,
        focus_speed: int = 4,
        image_stabilizer: bool = True,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fourcc: Optional[str] = None,
        open_video: bool = True,
    ):
        self.drive_speed = drive_speed
        self.focus_speed = focus_speed
        self._serial_lock = threading.Lock()
        self._ser = None
        self._cap = None
        self._server: Optional["CameraServer"] = None

        self._open_and_lock(port, baud)
        self._send(visca.IS_ON if image_stabilizer else visca.IS_OFF)
        if open_video:
            self._open_video(video_device, width, height, fourcc)

    # -- lifecycle -----------------------------------------------------

    def _open_and_lock(self, port: str, baud: int, retries: int = 5) -> None:
        # Right after USB re-enumeration the tty can exist but not be ready;
        # pyserial then raises a plain OSError from open(). Retry briefly.
        last_error = None
        for attempt in range(1, retries + 1):
            try:
                self._ser = serial.Serial(port, baud, timeout=0.5)
                last_error = None
                break
            except (serial.SerialException, OSError) as e:
                last_error = e
                if attempt < retries:
                    time.sleep(0.5)
        if last_error is not None:
            raise RuntimeError(
                f"could not open {port} after {retries} attempts: {last_error}"
            )
        try:
            fcntl.flock(self._ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self._ser.close()
            raise RuntimeError(f"{port} is already locked by another process")

    def _open_video(self, device, width, height, fourcc) -> None:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"could not open video device {device!r}")
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap = cap

    def close(self) -> None:
        if self._server is not None:
            self._server.stop()
            self._server = None
        if self._ser is not None:
            try:
                self.stop()
                fcntl.flock(self._ser.fileno(), fcntl.LOCK_UN)
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- video ---------------------------------------------------------

    def read(self) -> Optional[np.ndarray]:
        """Grab one BGR frame, or None if the grab failed. Don't call this
        while serving - the server's capture thread owns the device."""
        if self._cap is None:
            raise RuntimeError("video not opened")
        ok, frame = self._cap.read()
        return frame if ok else None

    def serve(self, host: str = "0.0.0.0", port: int = DEFAULT_PORT,
              jpeg_quality: int = 80, block: bool = True) -> "CameraServer":
        """Stream video and accept remote control. Blocks unless block=False."""
        if self._cap is None:
            raise RuntimeError("video not opened")
        if self._server is not None:
            raise RuntimeError("already serving")
        self._server = CameraServer(self, host, port, jpeg_quality)
        self._server.start()
        if block:
            try:
                self._server.wait()
            except KeyboardInterrupt:
                pass
            finally:
                self.close()
        return self._server

    # -- serial --------------------------------------------------------

    def _send(self, cmd: bytes, read_len: int = 64) -> bytes:
        # Drop any late reply left over from a previous timed-out read so it
        # can't be concatenated with this command's own reply.
        with self._serial_lock:
            self._ser.reset_input_buffer()
            self._ser.write(cmd)
            return self._ser.read(read_len)

    # -- zoom / focus --------------------------------------------------

    def zoom(self, direction: int) -> None:
        """Continuous zoom: >0 in, <0 out, 0 stop."""
        if direction > 0:
            self._send(visca.zoom_drive(self.drive_speed, "in"))
        elif direction < 0:
            self._send(visca.zoom_drive(self.drive_speed, "out"))
        else:
            self._send(visca.ZOOM_STOP)

    def set_zoom_position(self, position: float) -> None:
        """Absolute zoom, normalized 0.0 (wide) - 1.0 (full tele)."""
        position = max(0.0, min(1.0, position))
        self._send(visca.zoom_absolute(round(position * visca.ZOOM_MAX)))

    def reset_zoom(self) -> None:
        self._send(visca.zoom_absolute(0))

    def get_zoom(self) -> Optional[float]:
        """Normalized zoom position 0.0-1.0, or None if the read failed."""
        try:
            pos = visca.parse_zoom_inquiry(self._send(visca.ZOOM_INQUIRY, 16))
        except ValueError:
            return None
        return pos / visca.ZOOM_MAX

    def focus(self, direction: int) -> None:
        """Continuous focus: >0 far, <0 near, 0 stop."""
        if direction > 0:
            self._send(visca.focus_drive(self.focus_speed, "far"))
        elif direction < 0:
            self._send(visca.focus_drive(self.focus_speed, "near"))
        else:
            self._send(visca.FOCUS_STOP)

    def set_focus_auto(self, auto: bool) -> None:
        self._send(visca.FOCUS_AUTO if auto else visca.FOCUS_MANUAL)

    def set_focus_position(self, position: float) -> None:
        """Absolute focus, normalized 0.0 (near) - 1.0 (far)."""
        position = max(0.0, min(1.0, position))
        self._send(visca.focus_absolute(round(position * visca.FOCUS_MAX)))

    def get_focus(self) -> Optional[float]:
        try:
            pos = visca.parse_focus_inquiry(self._send(visca.FOCUS_INQUIRY, 16))
        except ValueError:
            return None
        return pos / visca.FOCUS_MAX

    def focus_one_push(self) -> None:
        self._send(visca.FOCUS_ONE_PUSH)

    def stop(self) -> None:
        """Stop any zoom/focus motion."""
        self._send(visca.ZOOM_STOP)
        self._send(visca.FOCUS_STOP)

    def set_image_stabilizer(self, on: bool) -> None:
        self._send(visca.IS_ON if on else visca.IS_OFF)

    # -- exposure ------------------------------------------------------

    def set_exposure_mode(self, mode: str) -> None:
        """'auto', 'shutter_priority', 'iris_priority' or 'manual'."""
        self._send(visca.ae_mode(mode))

    def set_shutter(self, value: int) -> None:
        self._send(visca.shutter_direct(value))

    def set_iris(self, value: int) -> None:
        self._send(visca.iris_direct(value))

    def set_gain(self, value: int) -> None:
        self._send(visca.gain_direct(value))

    def set_exposure_comp(self, value: int) -> None:
        self._send(visca.exposure_comp_direct(value))

    # -- white balance -------------------------------------------------

    def set_wb_mode(self, mode: str) -> None:
        """'auto', 'indoor', 'outdoor', 'onepush', 'atw' or 'manual'."""
        self._send(visca.wb_mode(mode))

    def set_wb_red_gain(self, value: int) -> None:
        self._send(visca.wb_red_gain(value))

    def set_wb_blue_gain(self, value: int) -> None:
        self._send(visca.wb_blue_gain(value))

    # -- IR / day-night ------------------------------------------------

    def set_icr(self, state: str) -> None:
        """Manual IR-cut filter: 'bw', 'color' or 'off'."""
        self._send(visca.icr(state))

    def set_auto_icr(self, state: str) -> None:
        self._send(visca.auto_icr(state))

    def set_auto_icr_threshold(self, value: int) -> None:
        self._send(visca.auto_icr_threshold(value))

    # -- dynamic range / low-light -------------------------------------

    def set_wdr(self, on: bool) -> None:
        self._send(visca.WDR_ON if on else visca.WDR_OFF)

    def set_defog(self, level: int) -> None:
        """0 off; 1-3 low/mid/high."""
        self._send(visca.defog(level))

    def set_backlight_compensation(self, on: bool) -> None:
        self._send(
            visca.BACKLIGHT_COMPENSATION_ON if on
            else visca.BACKLIGHT_COMPENSATION_OFF
        )

    def set_hlc(self, on: bool) -> None:
        self._send(visca.HLC_ON if on else visca.HLC_OFF)

    def set_high_sensitivity(self, on: bool) -> None:
        self._send(
            visca.HIGH_SENSITIVITY_ON if on else visca.HIGH_SENSITIVITY_OFF
        )


# ---------------------------------------------------------------------------
# Wire helpers
# ---------------------------------------------------------------------------


def _send_msg(sock: socket.socket, kind: int, payload: bytes) -> None:
    sock.sendall(_HEADER.pack(kind, len(payload)) + payload)


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("connection closed")
        buf += chunk
    return bytes(buf)


def _recv_msg(sock: socket.socket):
    kind, length = _HEADER.unpack(_recv_exact(sock, _HEADER.size))
    if length > _MAX_MSG:
        raise ConnectionError(f"message too large: {length}")
    return kind, _recv_exact(sock, length)


# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------


class CameraServer:
    def __init__(self, camera: NeoHDCamera, host: str, port: int,
                 jpeg_quality: int = 80):
        self._camera = camera
        self._quality = jpeg_quality
        self._stop = threading.Event()
        self._cond = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._seq = 0
        self._clients = 0
        self._threads = []

        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((host, port))
        self._listener.listen(4)
        self._listener.settimeout(0.5)
        self.address = self._listener.getsockname()

    def start(self) -> None:
        for target in (self._capture_loop, self._accept_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)

    def wait(self) -> None:
        while not self._stop.is_set():
            self._stop.wait(0.5)

    def stop(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        try:
            self._listener.close()
        except OSError:
            pass
        for t in self._threads:
            t.join(timeout=2.0)

    def _capture_loop(self) -> None:
        while not self._stop.is_set():
            with self._cond:
                idle = self._clients == 0
            if idle:  # don't burn CPU encoding for nobody
                time.sleep(0.05)
                continue
            frame = self._camera.read()
            if frame is None:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality]
            )
            if not ok:
                continue
            with self._cond:
                self._jpeg = buf.tobytes()
                self._seq += 1
                self._cond.notify_all()

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(
                target=self._serve_client, args=(conn,), daemon=True
            ).start()

    def _serve_client(self, conn: socket.socket) -> None:
        with self._cond:
            self._clients += 1
        closed = threading.Event()
        write_lock = threading.Lock()

        def send(kind, payload):
            with write_lock:
                _send_msg(conn, kind, payload)

        def video_out():
            last = 0
            try:
                while not (closed.is_set() or self._stop.is_set()):
                    with self._cond:
                        self._cond.wait_for(
                            lambda: self._seq != last or self._stop.is_set()
                            or closed.is_set(),
                            timeout=1.0,
                        )
                        if self._seq == last or self._jpeg is None:
                            continue
                        last, jpeg = self._seq, self._jpeg
                    send(MSG_FRAME, jpeg)
            except OSError:
                closed.set()

        threading.Thread(target=video_out, daemon=True).start()
        try:
            while not self._stop.is_set():
                kind, payload = _recv_msg(conn)
                if kind != MSG_COMMAND:
                    continue
                send(MSG_REPLY, json.dumps(self._execute(payload)).encode())
        except (OSError, ConnectionError):
            pass
        finally:
            closed.set()
            with self._cond:
                self._clients -= 1
                self._cond.notify_all()
            conn.close()
            try:  # don't leave the lens running if a client vanishes mid-drive
                self._camera.stop()
            except Exception:
                pass

    def _execute(self, payload: bytes) -> dict:
        req_id = None
        try:
            req = json.loads(payload)
            req_id = req.get("id")
            method = req["method"]
            if method not in CONTROL_METHODS:
                raise ValueError(f"unknown method: {method!r}")
            result = getattr(self._camera, method)(
                *req.get("args", []), **req.get("kwargs", {})
            )
            return {"id": req_id, "ok": True, "result": result}
        except Exception as e:
            return {"id": req_id, "ok": False, "error": f"{type(e).__name__}: {e}"}


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------


class NeoHDClient:
    """Remote view of a served camera: `read()` for video, and every method in
    CONTROL_METHODS (e.g. `zoom(1)`, `set_wb_mode("auto")`) as an RPC."""

    def __init__(self, host: str, port: int = DEFAULT_PORT,
                 timeout: float = 5.0):
        self._timeout = timeout
        self._sock = socket.create_connection((host, port), timeout=timeout)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.settimeout(None)
        self._send_lock = threading.Lock()
        self._cond = threading.Condition()
        self._frame: Optional[np.ndarray] = None
        self._frame_seq = 0
        self._replies = {}
        self._next_id = 0
        self._closed = False
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _recv_loop(self) -> None:
        try:
            while True:
                kind, payload = _recv_msg(self._sock)
                if kind == MSG_FRAME:
                    frame = cv2.imdecode(
                        np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR
                    )
                    if frame is not None:
                        with self._cond:
                            self._frame = frame
                            self._frame_seq += 1
                            self._cond.notify_all()
                elif kind == MSG_REPLY:
                    reply = json.loads(payload)
                    with self._cond:
                        self._replies[reply["id"]] = reply
                        self._cond.notify_all()
        except (OSError, ConnectionError, ValueError):
            pass
        finally:
            with self._cond:
                self._closed = True
                self._cond.notify_all()

    def read(self, timeout: Optional[float] = None,
             new: bool = True) -> Optional[np.ndarray]:
        """Latest frame. With new=True (default) waits for a frame newer than
        the last one returned; returns None on timeout or a closed link."""
        with self._cond:
            if new:
                seen = getattr(self, "_seen_seq", 0)
                self._cond.wait_for(
                    lambda: self._frame_seq != seen or self._closed, timeout
                )
                if self._frame_seq == seen:
                    return None
                self._seen_seq = self._frame_seq
            return self._frame

    def frames(self):
        """Iterate frames until the connection closes."""
        while True:
            frame = self.read(timeout=1.0)
            if frame is not None:
                yield frame
            elif self._closed:
                return

    def call(self, method: str, *args, **kwargs):
        with self._cond:
            if self._closed:
                raise ConnectionError("connection closed")
            self._next_id += 1
            req_id = self._next_id
        payload = json.dumps(
            {"id": req_id, "method": method, "args": args, "kwargs": kwargs}
        ).encode()
        with self._send_lock:
            _send_msg(self._sock, MSG_COMMAND, payload)
        with self._cond:
            if not self._cond.wait_for(
                lambda: req_id in self._replies or self._closed, self._timeout
            ) or req_id not in self._replies:
                raise TimeoutError(f"no reply to {method}")
            reply = self._replies.pop(req_id)
        if not reply["ok"]:
            raise RuntimeError(reply["error"])
        return reply["result"]

    def __getattr__(self, name):
        if name in CONTROL_METHODS:
            return lambda *a, **kw: self.call(name, *a, **kw)
        raise AttributeError(name)

    def close(self) -> None:
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

_VIEW_KEYS = """z/x zoom in/out   f/v focus far/near   c stop
a focus auto   m focus manual   o one-push AF   r reset zoom   q quit"""


def _view(host: str, port: int) -> None:
    print(_VIEW_KEYS)
    with NeoHDClient(host, port) as cli:
        cv2.namedWindow("NeoHD", cv2.WINDOW_NORMAL)
        for frame in cli.frames():
            cv2.imshow("NeoHD", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("z"):
                cli.zoom(1)
            elif key == ord("x"):
                cli.zoom(-1)
            elif key == ord("f"):
                cli.focus(1)
            elif key == ord("v"):
                cli.focus(-1)
            elif key == ord("c"):
                cli.stop()
            elif key == ord("a"):
                cli.set_focus_auto(True)
            elif key == ord("m"):
                cli.set_focus_auto(False)
            elif key == ord("o"):
                cli.focus_one_push()
            elif key == ord("r"):
                cli.reset_zoom()
        cli.stop()
    cv2.destroyAllWindows()


class CameraSource(Source):
    """Video frames -> timesync bus. This is the FrameAligner's trigger.

    Grabs frames through OpenCV/V4L2 and stamps each one on the monotonic
    clock between `grab()` and `retrieve()`, so the stamp is the buffer
    dequeue rather than the JPEG/YUV decode that follows it. Publishes
    `{"frame": n}` on the "cam" stream; the aligner emits exactly one
    frames.csv row per frame published here.

    With `out_dir` set, frames are recorded from a writer thread -- so
    encoding and disk I/O never delay the next grab -- into one continuous
    `video.avi` via `avi.MjpegAviWriter`, alongside a sidecar index mapping
    each camera frame number to its byte offset. There is no per-frame-file
    mode: the video feed is always a single AVI.

    If the writer thread falls behind, frames are dropped from the *video*
    (counted in `dropped_writes`) and never from the bus -- a gap in the
    imagery is recoverable, a gap in the timeline is not. Dropping stays
    honest: the AVI index records real frame numbers, so a drop leaves a gap
    in the numbering rather than silently shifting every later frame.

    `stamp_frame_number` burns the frame number into the top-left corner
    before encoding, so the correspondence to frames.csv is checkable by eye.
    It is drawn on the copy that gets encoded; the bus timestamp is taken
    before any of this.

    Recording needs no VISCA control, so no serial port is opened by default;
    pass `visca_port` to drive the lens through NeoHDCamera at the same time.
    """

    RETRY_S = 1.0
    MAX_CONSECUTIVE_FAILURES = 30

    def __init__(self, bus, clock, device, out_dir=None, visca_port=None,
                 width=None, height=None, fourcc=None, fps=None,
                 latency_ms=0.0, jpeg_quality=90,
                 stamp_frame_number=True, queue_size=256, name="cam"):
        super().__init__(name, bus, clock)
        if isinstance(device, str):
            device = int(device) if device.isdigit() else device
        self.device = device
        self.out_dir = out_dir
        self.visca_port = visca_port
        self.width = width
        self.height = height
        self.fourcc = fourcc
        self.fps = fps
        self.latency_ns = int(latency_ms * 1_000_000)
        self.jpeg_quality = int(jpeg_quality)
        self.stamp_frame_number = stamp_frame_number

        self._cap = None
        self._cam = None
        self._actual = {}
        self._seq = 0
        self._consecutive_failures = 0
        self._next_retry = 0.0
        self._rows = queue.Queue(maxsize=queue_size)
        self._writer = None
        self._avi = None
        self._avi_path = None
        self._size_mismatch = 0
        self._encode_failures = 0
        self._t_first = None
        self._t_last = None

        self.frames = 0
        self.grab_failures = 0
        self.decode_failures = 0
        self.dropped_writes = 0
        self.reopens = 0

    # ---------------------------------------------------------------- capture

    def _open(self):
        if self.visca_port:
            self._cam = NeoHDCamera(self.visca_port, self.device,
                                    width=self.width, height=self.height,
                                    fourcc=self.fourcc)
            self._cap = self._cam._cap
        else:
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError(f"could not open video device "
                                   f"{self.device!r}")
            if self.fourcc:
                cap.set(cv2.CAP_PROP_FOURCC,
                        cv2.VideoWriter_fourcc(*self.fourcc))
            if self.width:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            if self.height:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if self.fps:
                cap.set(cv2.CAP_PROP_FPS, self.fps)
            self._cap = cap
        self._actual = self._read_actual()

    def _read_actual(self):
        cap = self._cap
        if cap is None:
            return {}
        try:
            cc = int(cap.get(cv2.CAP_PROP_FOURCC))
            return {
                "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                "fps": cap.get(cv2.CAP_PROP_FPS),
                "fourcc": ("".join(chr((cc >> (8 * i)) & 0xFF)
                                   for i in range(4)) if cc else None),
            }
        except cv2.error:
            return {}

    def _close(self):
        if self._cam is not None:
            try:
                self._cam.close()
            except Exception:
                pass
            self._cam = None
            self._cap = None
            return
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def setup(self):
        self._open()
        if self.out_dir:
            os.makedirs(self.out_dir, exist_ok=True)
            self._writer = threading.Thread(target=self._write_frames,
                                            name=f"{self.name}-jpeg",
                                            daemon=True)
            self._writer.start()
        self._write_info()

    def poll(self):
        if self._cap is None:
            now = time.monotonic()
            if now < self._next_retry:
                self._stop_evt.wait(min(0.05, self._next_retry - now))
                return
            self._open()
            self.reopens += 1
            self._consecutive_failures = 0
            return

        # grab() returns once the driver hands over the filled buffer, so the
        # stamp goes here -- before retrieve() spends time decoding it.
        ok = self._cap.grab()
        t_grab = self.clock.mono_ns()
        if not ok:
            self.grab_failures += 1
            self._on_failure()
            return
        ok, frame = self._cap.retrieve()
        if not ok or frame is None:
            self.decode_failures += 1
            self._on_failure()
            return

        self._consecutive_failures = 0
        self._seq += 1
        self.frames += 1
        t_host = t_grab - self.latency_ns
        if self._t_first is None:
            self._t_first = t_host
        self._t_last = t_host

        self.bus.publish(Record(stream=self.name, t_host_ns=t_host,
                                t_arrival_ns=t_grab,
                                fields={"frame": self._seq},
                                seq=self._seq))
        if self._writer is not None:
            try:
                self._rows.put_nowait((self._seq, t_host, frame))
            except queue.Full:
                self.dropped_writes += 1

    def _on_failure(self):
        self._consecutive_failures += 1
        if self._consecutive_failures >= self.MAX_CONSECUTIVE_FAILURES:
            print(f"[{self.name}] {self._consecutive_failures} failed grabs; "
                  f"reopening {self.device!r}", file=sys.stderr, flush=True)
            self._close()
            self._next_retry = time.monotonic() + self.RETRY_S
        else:
            self._stop_evt.wait(0.005)

    def teardown(self):
        self._close()
        if self._writer is not None:
            self._rows.put(None)
            # The thread finalises the AVI (idx1 + header patch) on its way
            # out, so give it room to finish rather than leaving a file that
            # needs avi.repair().
            self._writer.join(timeout=30.0)
            if self._writer.is_alive():
                print(f"[{self.name}] writer thread still draining; "
                      f"{self._avi_path} may need avi.repair()",
                      file=sys.stderr, flush=True)
        self._write_info()

    # ----------------------------------------------------------------- output

    def _write_frames(self):
        params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        try:
            while True:
                item = self._rows.get()
                if item is None:
                    return
                n, t_ns, frame = item
                if self.stamp_frame_number:
                    # On a copy: the caller may still hold the original, and
                    # the bus stamp was taken long before this.
                    frame = frame.copy()
                    self._stamp(frame, n)
                try:
                    ok, buf = cv2.imencode(".jpg", frame, params)
                except cv2.error:
                    ok = False
                if not ok:
                    self._encode_failures += 1
                    continue
                self._avi_write(buf.tobytes(), n, t_ns, frame.shape)
        except Exception:
            # Without this the thread would die quietly and every later frame
            # would just become a dropped_write, with nothing saying why.
            self._report("writer")
        finally:
            if self._avi is not None:
                try:
                    self._avi.close()
                except Exception:
                    pass

    def _avi_write(self, jpeg, n, t_ns, shape):
        h, w = shape[:2]
        if self._avi is None:
            # Size comes from the first real frame, not from CAP_PROP_*,
            # which some drivers report before negotiation settles.
            self._avi_path = os.path.join(self.out_dir, "video.avi")
            self._avi = MjpegAviWriter(self._avi_path, w, h,
                                       fps=self._nominal_fps())
        if (w, h) != (self._avi.width, self._avi.height):
            # AVI is fixed-size; a resize here would silently corrupt the
            # geometry of every later frame, so skip and count instead.
            self._size_mismatch += 1
            return
        try:
            self._avi.write(jpeg, frame=n, t_ns=t_ns)
        except (OSError, ValueError):
            self.dropped_writes += 1

    def _nominal_fps(self):
        for v in (self.fps, (self._actual or {}).get("fps")):
            if v:
                return float(v)
        return 30.0

    @staticmethod
    def _stamp(frame, n):
        text = str(n)
        cv2.putText(frame, text, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, text, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 255), 1, cv2.LINE_AA)

    def measured_fps(self):
        if self.frames < 2 or self._t_first is None:
            return None
        span = (self._t_last - self._t_first) / 1e9
        return round((self.frames - 1) / span, 2) if span > 0 else None

    def _write_info(self):
        if not self.out_dir:
            return
        actual = self._actual or self._read_actual()
        info = {
            "device": self.device,
            "visca_port": self.visca_port,
            "requested": {"width": self.width, "height": self.height,
                          "fourcc": self.fourcc, "fps": self.fps},
            "actual": actual,
            "latency_ns": self.latency_ns,
            "jpeg_quality": self.jpeg_quality,
            "stamp": "CLOCK_MONOTONIC between grab() and retrieve(), "
                     "minus latency_ns",
            "frames": self.frames,
            "fps_measured": self.measured_fps(),
            "grab_failures": self.grab_failures,
            "decode_failures": self.decode_failures,
            "dropped_writes": self.dropped_writes,
            "encode_failures": self._encode_failures,
            "size_mismatch": self._size_mismatch,
            "stamp_frame_number": self.stamp_frame_number,
            "avi": self._avi_path,
            "avi_frames": self._avi.frames if self._avi else 0,
            "reopens": self.reopens,
            "errors": self.errors,
        }
        path = os.path.join(os.path.dirname(self.out_dir.rstrip("/")) or ".",
                            "cam_info.json")
        with open(path, "w") as f:
            json.dump(info, f, indent=2)
            f.flush()
            os.fsync(f.fileno())


def main(argv=None):
    import argparse

    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("serve", help="stream video + accept control over TCP")
    s.add_argument("--serial", default="/dev/ttyACM0")
    s.add_argument("--video", default="0",
                   help="video device index or path (default 0)")
    s.add_argument("--host", default="0.0.0.0")
    s.add_argument("--port", type=int, default=DEFAULT_PORT)
    s.add_argument("--width", type=int)
    s.add_argument("--height", type=int)
    s.add_argument("--fourcc", help="e.g. YUYV, MJPG")
    s.add_argument("--quality", type=int, default=80, help="JPEG quality")

    v = sub.add_parser("view", help="view a served camera and drive it")
    v.add_argument("host")
    v.add_argument("--port", type=int, default=DEFAULT_PORT)

    args = p.parse_args(argv)
    if args.cmd == "view":
        _view(args.host, args.port)
        return
    video = int(args.video) if args.video.isdigit() else args.video
    try:
        cam = NeoHDCamera(
            args.serial, video, width=args.width, height=args.height,
            fourcc=args.fourcc,
        )
    except RuntimeError as e:
        print(e, file=sys.stderr)
        sys.exit(1)
    print(f"serving on {args.host}:{args.port}")
    cam.serve(args.host, args.port, jpeg_quality=args.quality)


if __name__ == "__main__":
    main()
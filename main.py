import argparse
import importlib
import json
import os
import signal
import sys
import time
from datetime import datetime, timezone

from aligner import (EVENT, INTERP, NEAREST, FrameAligner, StreamSpec,
                     angle_lerp_deg, angle_lerp_rad, lerp)
from bus import Bus
from clock import Clock
from csvlog import CsvLog

MS = 1_000_000
CAM_COLUMNS = ["frame"]


def sweep_columns(n_sectors):
    return (["id", "n_points", "n_valid", "sweep_ms", "dir", "angle_lo",
             "angle_hi", "range_min", "temp_c"]
            + [f"s{i:02d}" for i in range(n_sectors)])


def stream_specs(n_sectors):
    return [
        StreamSpec("fcu.att",
                   ["roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"],
                   INTERP,
                   {"roll": lerp, "pitch": lerp, "yaw": angle_lerp_rad,
                    "rollspeed": lerp, "pitchspeed": lerp, "yawspeed": lerp},
                   max_gap_ns=200 * MS),
        StreamSpec("fcu.gpos",
                   ["lat", "lon", "alt", "relative_alt", "vx", "vy", "vz", "hdg"],
                   INTERP,
                   {"lat": lerp, "lon": lerp, "alt": lerp, "relative_alt": lerp,
                    "vx": lerp, "vy": lerp, "vz": lerp, "hdg": angle_lerp_deg},
                   max_gap_ns=500 * MS),
        StreamSpec("lidar.sweep", sweep_columns(n_sectors), EVENT,
                   max_gap_ns=1500 * MS),
        StreamSpec("lidar.pt", ["angle_deg", "dist_m", "x_m", "y_m", "sweep"],
                   NEAREST,
                   max_gap_ns=50 * MS),
    ]


def load_source(module, cls, flag, stage):
    try:
        mod = importlib.import_module(f"sources.{module}")
    except ModuleNotFoundError as e:
        if e.name == f"sources.{module}":
            sys.exit(f"{flag} needs sources/{module}.py ({stage}), "
                     f"which is not written yet")
        raise
    return getattr(mod, cls)


def build_sources(args, bus, clock, run_dir):
    sources = []
    if args.camera:
        Camera = load_source("camera", "CameraSource", "--camera", "stage 7")
        sources.append(Camera(bus, clock, args.camera,
                              os.path.join(run_dir, "video"),
                              stamp_frame_number=not args.no_stamp,
                              jpeg_quality=args.jpeg_quality))
    if args.fcu:
        Mavlink = load_source("mavlink", "MavlinkSource", "--fcu", "stage 6")
        sources.append(Mavlink(bus, clock, args.fcu, args.baud, out_dir=run_dir))
    if args.sf45:
        Lidar = load_source("lidar_driver", "LidarSource", "--sf45", "stage 5")
        sources.append(Lidar(bus, clock, args.sf45, args.sf45_baud,
                             low_angle=args.low, high_angle=args.high,
                             n_sectors=args.sectors,
                             update_rate=args.sf45_rate, out_dir=run_dir))
    return sources


def write_manifest(path, clock, args, columns):
    with open(path, "w") as f:
        json.dump({
            "boot_mono_ns": clock.boot_time,
            "boot_unix_ns": clock.boot_unix_time,
            "capture_uncertainty_ns": clock.capture_uncertainty_ns,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "args": vars(args),
            "columns": columns,
        }, f, indent=2)
        f.flush()
        os.fsync(f.fileno())


def status(log, fa, bus, sources):
    st = bus.stats()
    parts = [f"frames={log.rows}", f"pending={fa.pending}",
             f"timed_out={fa.rows_timed_out}",
             f"dropped={sum(st['dropped'].values())}", f"qdepth={st['depth']}"]
    parts += [f"{k}={v}" for k, v in sorted(st["published"].items())]
    for s in sources:
        if s.failed:
            parts.append(f"{s.name}:FAILED")
        elif not s.is_alive():
            parts.append(f"{s.name}:STOPPED")
        elif s.errors:
            parts.append(f"{s.name}:err={s.errors}")
    print("  ".join(parts), file=sys.stderr, flush=True)


def main():
    p = argparse.ArgumentParser(
        description="Record camera frames with time-aligned FCU and LiDAR data")
    p.add_argument("--out", default="runs")
    p.add_argument("--camera", help="camera device, e.g. /dev/video2")
    p.add_argument("--fcu", help="pymavlink connection, e.g. udpin:0.0.0.0:14550 "
                   "(MAVProxy --out) or /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--sf45", help="SF45/B serial port, e.g. /dev/ttyUSB0")
    p.add_argument("--sf45-baud", type=int, default=115200)
    p.add_argument("--sf45-rate", type=int, default=6,
                   help="update rate setting 1-7; on this unit 3 = 200, "
                        "4 = 400 and 5-7 = ~478 points/s")
    p.add_argument("--low", type=float, default=None,
                   help="scan low angle, -160..-10 (default: keep the device's)")
    p.add_argument("--high", type=float, default=None,
                   help="scan high angle, 10..160 (default: keep the device's)")
    p.add_argument("--sectors", type=int, default=12,
                   help="LiDAR sweep summary sectors; sector i covers "
                        "low + i*(high-low)/sectors, edges are in "
                        "lidar_info.json")
    p.add_argument("--max-wait-ms", type=int, default=600,
                   help="hard cap on how long a frame waits for slow streams")
    p.add_argument("--cam-offset-ms", type=float, default=0.0,
                   help="added to each frame stamp before lookup")
    p.add_argument("--no-stamp", action="store_true",
                   help="do not burn the frame number into the image")
    p.add_argument("--quality", type=int, default=90, dest="jpeg_quality",
                   help="MJPEG quality 1-100; lower means a smaller file")
    p.add_argument("--status-s", type=float, default=5.0)
    args = p.parse_args()

    if not (args.camera or args.fcu or args.sf45):
        p.error("nothing to record: give at least one of --camera/--fcu/--sf45")
    if not args.camera:
        print("warning: no --camera, so no frames and frames.csv stays empty",
              file=sys.stderr)

    clock = Clock()
    run_dir = os.path.join(
        args.out, datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    os.makedirs(run_dir)
    bus = Bus()
    fa = FrameAligner(stream_specs(args.sectors), trigger="cam",
                      trigger_columns=CAM_COLUMNS,
                      max_wait_ns=args.max_wait_ms * MS,
                      frame_offset_ns=int(args.cam_offset_ms * MS),
                      to_unix_ns=clock.to_unix_ns)
    log = CsvLog(os.path.join(run_dir, "frames.csv"), fa.columns())
    write_manifest(os.path.join(run_dir, "manifest.json"), clock, args,
                   fa.columns())
    sources = build_sources(args, bus, clock, run_dir)

    stop = False
    def on_signal(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    for s in sources:
        s.start()
    print(f"recording -> {run_dir}   (ctrl-c to stop)", file=sys.stderr)

    next_status = time.monotonic() + args.status_s

    try:
        while not stop:
            rec = bus.get(timeout=0.05)
            n = 0
            while rec is not None:
                fa.add(rec)
                n += 1
                rec = bus.get(timeout=0) if n < 1000 else None
            log.write_many(fa.drain(clock.mono_ns()))
            if time.monotonic() >= next_status:
                status(log, fa, bus, sources)
                next_status += args.status_s

    finally:
        for s in sources:
            s.stop()
        for s in sources:
            s.join(timeout=3.0)

        while (rec := bus.get(timeout=0)) is not None:
            fa.add(rec)
        log.write_many(fa.drain(clock.mono_ns()))
        log.write_many(fa.finalize())
        log.close()
        status(log, fa, bus, sources)
        print(f"wrote {log.rows} frames -> {log.path}", file=sys.stderr)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Read a run's data and its video together, while both are still growing.

`main.py --camera ...` appends to three files at once:
`frames.csv` (one aligned row per frame), `video/video.avi`, and
`video/video_index.csv` (frame number -> byte offset). This reads all three
live and joins them on the frame number, so you can watch -- or verify -- the
imagery next to the FCU and LiDAR data as it is recorded.

    python3 follow.py                      # newest run under runs/, verify
    python3 follow.py --show               # live window, data drawn on frame
    python3 follow.py --save annotated.avi # one AVI with the data burnt in
    python3 follow.py --fields fcu.att.roll,lidar.sweep.n_valid

Two ordering facts shape this, both of them normal rather than errors:

* frames.csv *lags* the video. The aligner holds each frame up to
  `--max-wait-ms` waiting for slow streams, while the writer thread indexes
  the JPEG straight away. So a data row's frame is almost always already in
  the index; `--wait-ms` covers the case where the writer is behind instead.
* A frame can be missing from the video for good: if the writer queue
  overflows, the frame is dropped from the imagery but never from the
  timeline. Those are reported as `missing`, not as failures.

Partial lines are expected -- this reads between the writer's flushes -- and
are retried rather than treated as corruption.
"""
import argparse
import csv
import io
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from avi import MjpegAviReader, MjpegAviWriter  # noqa: E402

DEFAULT_FIELDS = ("fcu.att.roll", "fcu.att.pitch", "fcu.att.yaw",
                  "lidar.sweep.n_valid", "lidar.pt.dist_m")


class CsvTail:
    """Tail a CSV that another process is appending to.

    Only whole lines are parsed; a trailing partial line is kept and
    completed on a later poll. The header is read once, however late it
    appears.
    """

    def __init__(self, path):
        self.path = path
        self._pos = 0
        self._buf = ""
        self.columns = None
        self.partial_reads = 0

    def poll(self):
        rows = []
        try:
            with open(self.path, "r", newline="") as f:
                f.seek(self._pos)
                chunk = f.read()
                self._pos = f.tell()
        except FileNotFoundError:
            return rows
        if not chunk:
            return rows
        self._buf += chunk
        cut = self._buf.rfind("\n")
        if cut == -1:
            self.partial_reads += 1
            return rows
        whole, self._buf = self._buf[:cut], self._buf[cut + 1:]
        if self._buf:
            self.partial_reads += 1
        for line in whole.split("\n"):
            if not line:
                continue
            if self.columns is None:
                self.columns = next(csv.reader([line]))
                continue
            vals = next(csv.reader([line]))
            if len(vals) != len(self.columns):
                # Writer was mid-row when we read; the remainder arrives next
                # poll as its own line, so this one is genuinely unusable.
                self.partial_reads += 1
                continue
            rows.append(dict(zip(self.columns, vals)))
        return rows


def newest_run(root, wait_s=15.0):
    """Newest run dir, waiting for one to appear.

    The follower is usually started alongside the recorder, so the directory
    may not exist yet for a moment.
    """
    t0 = time.monotonic()
    while True:
        runs = []
        if os.path.isdir(root):
            runs = [os.path.join(root, d) for d in os.listdir(root)
                    if os.path.isdir(os.path.join(root, d))]
        if runs:
            return max(runs, key=os.path.getmtime)
        if time.monotonic() - t0 > wait_s:
            sys.exit(f"no run directory appeared under {root} in {wait_s:.0f}s")
        time.sleep(0.1)


def fmt(v, width=9):
    if v in (None, "", "None"):
        return "-".rjust(width)
    try:
        return f"{float(v):>{width}.3f}"
    except (TypeError, ValueError):
        return str(v)[:width].rjust(width)


def draw(img, row, fields):
    import cv2
    y = img.shape[0] - 8 - 18 * len(fields)
    cv2.rectangle(img, (0, y - 18), (360, img.shape[0]), (0, 0, 0), -1)
    cv2.putText(img, f"frame {row.get('frame')}  t={row.get('t_unix', '')}",
                (6, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 255, 120), 1,
                cv2.LINE_AA)
    for i, f in enumerate(fields):
        cv2.putText(img, f"{f:<22s}{fmt(row.get(f))}", (6, y + 14 + 18 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1,
                    cv2.LINE_AA)
    return img


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run", nargs="?", help="run directory (default: newest under --root)")
    p.add_argument("--root", default="runs")
    p.add_argument("--show", action="store_true", help="live window (needs a display)")
    p.add_argument("--save", metavar="AVI",
                   help="write one continuous AVI with the data drawn on each "
                        "frame (readable while it grows, like the recording)")
    p.add_argument("--fields", default=",".join(DEFAULT_FIELDS))
    p.add_argument("--wait-ms", type=float, default=1500.0,
                   help="how long to wait for a frame to appear in the video "
                        "index before calling it missing")
    p.add_argument("--poll-ms", type=float, default=50.0)
    p.add_argument("--timeout-s", type=float, default=0.0,
                   help="stop after this long with no new data (0 = forever)")
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args()

    run = args.run or newest_run(args.root)
    frames_csv = os.path.join(run, "frames.csv")
    avi = os.path.join(run, "video", "video.avi")
    fields = [f for f in args.fields.split(",") if f]
    print(f"following {run}", file=sys.stderr)

    for path in (frames_csv, avi):
        t0 = time.monotonic()
        while not os.path.exists(path):
            if time.monotonic() - t0 > 10:
                sys.exit(f"{path} never appeared (is the recorder running "
                         f"with --camera?)")
            time.sleep(0.1)

    if args.save and os.path.isdir(args.save):
        sys.exit(f"--save takes a file path now, not a directory: {args.save}")
    writer = None   # opened on the first frame, which gives us the size

    tail = CsvTail(frames_csv)
    reader = MjpegAviReader(avi)
    pending = []          # rows whose frame is not in the index yet
    joined = missing = decode_errors = 0
    last_data = time.monotonic()

    try:
        while True:
            new = tail.poll()
            if new:
                last_data = time.monotonic()
                pending.extend((time.monotonic(), r) for r in new)
            if pending:
                reader.refresh()

            still = []
            for seen_at, row in pending:
                try:
                    n = int(row["frame"])
                except (KeyError, TypeError, ValueError):
                    continue
                if n not in reader._by_frame:
                    if (time.monotonic() - seen_at) * 1000 < args.wait_ms:
                        still.append((seen_at, row))
                    else:
                        missing += 1
                        if not args.quiet:
                            print(f"frame {n}: not in video index "
                                  f"(dropped from imagery)", file=sys.stderr)
                    continue
                try:
                    img = reader.frame(n)
                except (IOError, KeyError) as e:
                    decode_errors += 1
                    print(f"frame {n}: {e}", file=sys.stderr)
                    continue
                joined += 1
                if args.show or args.save:
                    import cv2
                    out = draw(img, row, fields)
                    if args.save:
                        if writer is None:
                            h, w = out.shape[:2]
                            writer = MjpegAviWriter(args.save, w, h, 30.0)
                        ok, buf = cv2.imencode(
                            ".jpg", out,
                            [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                        if ok:
                            # Keep the recorder's frame numbers, so the
                            # annotated file indexes the same way.
                            writer.write(buf.tobytes(), frame=n,
                                         t_ns=int(row.get("t_ns") or 0))
                    if args.show:
                        cv2.imshow("timesync", out)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            raise KeyboardInterrupt
                elif not args.quiet and joined % 30 == 1:
                    vals = "  ".join(f"{f}={fmt(row.get(f))}" for f in fields)
                    print(f"frame {n:>6}  {img.shape[1]}x{img.shape[0]}  "
                          f"{vals}")
            pending = still

            if args.timeout_s and time.monotonic() - last_data > args.timeout_s:
                break
            time.sleep(args.poll_ms / 1000.0)
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.close()
            print(f"wrote {writer.frames} annotated frames -> {args.save}",
                  file=sys.stderr)
        reader.close()
        if args.show:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception:
                pass

    print(f"joined={joined}  missing_from_video={missing}  "
          f"decode_errors={decode_errors}  "
          f"partial_csv_reads={tail.partial_reads}  "
          f"partial_index_reads={reader.partial_reads}", file=sys.stderr)
    return 1 if decode_errors else 0


if __name__ == "__main__":
    sys.exit(main())

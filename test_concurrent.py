#!/usr/bin/env python3
"""Stress the append-while-reading protocol in avi.py.

The live recorder polls slowly enough that it rarely lands mid-flush, so it
does not really test the hazard. This does: it writes as fast as it can while
several readers hammer the index and the file, and it verifies *content*
rather than just checking that calls return. Each frame is filled with a
colour derived from its own frame number, so a torn read, a stale index row
or an off-by-one offset decodes to the wrong colour and is caught -- failures
that "did it raise?" testing would miss entirely.

    python3 test_concurrent.py [--frames 600] [--readers 4]

Covers: concurrent append+read, torn index lines, reading the frame that is
being written, random access on a growing file, and recovery of a file whose
writer was killed before finalising.
"""
import argparse
import os
import random
import shutil
import subprocess
import sys
import tempfile
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from avi import MjpegAviReader, MjpegAviWriter, repair  # noqa: E402

W, H = 192, 144


def colour(n):
    """A colour that is a pure function of the frame number."""
    return ((n * 37) % 256, (n * 91) % 256, (n * 13) % 256)


def make(n):
    f = np.empty((H, W, 3), np.uint8)
    f[:] = colour(n)
    return f


def check(img, n, tol=12):
    """Does this decoded image really hold frame n's colour?"""
    if img is None or img.shape != (H, W, 3):
        return False, f"shape {None if img is None else img.shape}"
    want = np.array(colour(n), dtype=np.float64)
    got = img.reshape(-1, 3).mean(axis=0)
    # JPEG is lossy, so compare means within a tolerance rather than exactly.
    if np.abs(got - want).max() > tol:
        return False, f"colour {got.round(1).tolist()} != {want.tolist()}"
    return True, ""


class Stats:
    def __init__(self):
        self.lock = threading.Lock()
        self.verified = 0
        self.errors = []
        self.mismatches = 0

    def ok(self, k=1):
        with self.lock:
            self.verified += k

    def bad(self, msg, mismatch=False):
        with self.lock:
            if mismatch:
                self.mismatches += 1
            if len(self.errors) < 40:
                self.errors.append(msg)


def writer_thread(path, total, stats, done, quality):
    enc = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    try:
        w = MjpegAviWriter(path, W, H, 30.0)
        for n in range(1, total + 1):
            ok, buf = cv2.imencode(".jpg", make(n), enc)
            if not ok:
                stats.bad(f"writer: encode failed at {n}")
                continue
            w.write(buf.tobytes(), frame=n, t_ns=n * 1_000_000)
        w.close()
    except Exception as e:
        stats.bad(f"writer: {type(e).__name__}: {e}")
    finally:
        done.set()


def random_reader(path, stats, done, name):
    """Random access into whatever is indexed so far, verifying content."""
    try:
        r = MjpegAviReader(path)
    except Exception as e:
        stats.bad(f"{name}: open: {type(e).__name__}: {e}")
        return
    rnd = random.Random(hash(name) & 0xFFFF)
    try:
        while not done.is_set():
            r.refresh()
            if not len(r):
                continue
            for _ in range(8):
                n = rnd.choice(r.frames)
                try:
                    img = r.frame(n)
                except Exception as e:
                    stats.bad(f"{name}: frame {n}: {type(e).__name__}: {e}")
                    continue
                good, why = check(img, n)
                if good:
                    stats.ok()
                else:
                    stats.bad(f"{name}: frame {n} WRONG CONTENT: {why}", True)
        r.refresh()
    finally:
        r.close()
    return r.partial_reads


def tail_reader(path, total, stats, done, out):
    """Read every frame exactly once, in order, as it appears."""
    r = MjpegAviReader(path)
    seen = 0
    try:
        while True:
            added = r.refresh()
            frames = r.frames
            while seen < len(frames):
                n = frames[seen]
                if n != seen + 1:
                    stats.bad(f"tail: index out of order at position {seen}: "
                              f"frame {n}", True)
                try:
                    img = r.frame(n)
                except Exception as e:
                    stats.bad(f"tail: frame {n}: {type(e).__name__}: {e}")
                    seen += 1
                    continue
                good, why = check(img, n)
                if good:
                    stats.ok()
                else:
                    stats.bad(f"tail: frame {n} WRONG CONTENT: {why}", True)
                seen += 1
            if seen >= total:
                break
            # Only conclude the stream has ended after a refresh that found
            # nothing AND the writer being finished. Breaking as soon as
            # `done` is set would drop every row the writer flushed after
            # this iteration's refresh -- which is exactly how this test
            # first reported 287 of 600 frames.
            if done.is_set() and added == 0:
                break
            if not added:
                time.sleep(0.0005)
    finally:
        out["partial"] = r.partial_reads
        out["seen"] = seen
        r.close()


def run_live(tmp, total, readers, quality):
    path = os.path.join(tmp, "stress.avi")
    stats = Stats()
    done = threading.Event()
    tail_out = {}

    threads = [threading.Thread(target=writer_thread,
                                args=(path, total, stats, done, quality),
                                name="writer")]
    # Give the writer a head start so the readers have a file to open.
    threads[0].start()
    t0 = time.monotonic()
    while not os.path.exists(path) and time.monotonic() - t0 < 5:
        time.sleep(0.005)

    rt = threading.Thread(target=tail_reader,
                          args=(path, total, stats, done, tail_out),
                          name="tail")
    rt.start()
    threads.append(rt)
    for i in range(readers):
        t = threading.Thread(target=random_reader,
                             args=(path, stats, done, f"rnd{i}"),
                             name=f"rnd{i}")
        t.start()
        threads.append(t)
    for t in threads:
        t.join(timeout=180)

    print(f"  frames written          : {total}")
    print(f"  content checks passed   : {stats.verified}")
    print(f"  wrong-content failures  : {stats.mismatches}")
    print(f"  tail read in order      : {tail_out.get('seen')} / {total}")
    print(f"  torn index lines seen   : {tail_out.get('partial')} "
          f"(retried, not errors)")
    if stats.errors:
        print("  FAILURES:")
        for e in stats.errors:
            print("   ", e)
    return stats, tail_out, path


def validate_final(path, total):
    """The finished file must be a standard AVI, not just one we can read."""
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-count_frames",
         "-show_entries", "stream=codec_name,width,height,nb_read_frames",
         "-of", "default=nw=1", path],
        capture_output=True, text=True)
    info = dict(l.split("=", 1) for l in out.stdout.strip().splitlines()
                if "=" in l)
    ok = (info.get("codec_name") == "mjpeg"
          and int(info.get("nb_read_frames", 0)) == total
          and int(info.get("width", 0)) == W
          and int(info.get("height", 0)) == H)
    print(f"  ffprobe                 : {info} -> {'OK' if ok else 'MISMATCH'}")
    if out.stderr.strip():
        print(f"  ffprobe stderr          : {out.stderr.strip()}")

    c = cv2.VideoCapture(path)
    n = 0
    bad = 0
    while True:
        got, img = c.read()
        if not got:
            break
        n += 1
        good, _ = check(img, n)
        if not good:
            bad += 1
    c.release()
    print(f"  cv2 sequential decode   : {n} frames, {bad} wrong content "
          f"-> {'OK' if n == total and bad == 0 else 'MISMATCH'}")
    return ok and n == total and bad == 0


def run_crash(tmp, total, quality):
    """A writer killed before close() must still be fully recoverable."""
    path = os.path.join(tmp, "crash.avi")
    enc = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    w = MjpegAviWriter(path, W, H, 30.0)
    for n in range(1, total + 1):
        ok, buf = cv2.imencode(".jpg", make(n), enc)
        w.write(buf.tobytes(), frame=n, t_ns=n * 1_000_000)
    # Abandon the writer without close(): no idx1, header counters still zero.
    w._f.flush()
    os.fsync(w._f.fileno())
    w._idx.flush()
    os.fsync(w._idx.fileno())
    size_before = os.path.getsize(path)

    pre = cv2.VideoCapture(path)
    pre_n = 0
    while True:
        got, _ = pre.read()
        if not got:
            break
        pre_n += 1
    pre.release()

    # The sidecar index still works on the unfinalised file.
    r = MjpegAviReader(path)
    idx_n = len(r)
    bad = 0
    for n in r.frames:
        good, _ = check(r.frame(n), n)
        bad += 0 if good else 1
    r.close()

    recovered = repair(path)
    post = cv2.VideoCapture(path)
    post_n = 0
    post_bad = 0
    while True:
        got, img = post.read()
        if not got:
            break
        post_n += 1
        good, _ = check(img, post_n)
        post_bad += 0 if good else 1
    post.release()

    print(f"  unfinalised size        : {size_before} bytes")
    print(f"  index still readable    : {idx_n}/{total} frames, {bad} wrong")
    print(f"  cv2 before repair       : {pre_n} frames")
    print(f"  repair() rebuilt        : {recovered} entries")
    print(f"  cv2 after repair        : {post_n} frames, {post_bad} wrong "
          f"-> {'OK' if post_n == total and post_bad == 0 else 'MISMATCH'}")
    return idx_n == total and bad == 0 and post_n == total and post_bad == 0


def run_torn(tmp, quality):
    """A half-written index line must be skipped, then picked up when complete.

    Real runs rarely produce one: each index row is a single small write
    followed by a flush, so a reader almost always sees whole lines (hence
    "torn index lines seen: 0" above). That is a property of the write
    pattern, not proof the reader copes -- so force the condition here.
    """
    path = os.path.join(tmp, "torn.avi")
    enc = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    w = MjpegAviWriter(path, W, H, 30.0)
    for n in range(1, 11):
        ok, buf = cv2.imencode(".jpg", make(n), enc)
        w.write(buf.tobytes(), frame=n, t_ns=n * 1_000_000)
    w.close()

    with open(w.index_path) as f:
        text = f.read()
    lines = text.split("\n")
    while lines and not lines[-1]:
        lines.pop()
    last = lines[-1]
    head = "\n".join(lines[:-1]) + "\n"
    cut = len(last) // 2

    # Writer caught mid-line: 9 whole rows plus a fragment, no newline.
    with open(w.index_path, "w") as f:
        f.write(head + last[:cut])

    r = MjpegAviReader(path)
    n_partial_seen = len(r)
    partial_flagged = r.partial_reads
    frag_ok = True
    for n in r.frames:
        good, _ = check(r.frame(n), n)
        frag_ok = frag_ok and good

    # The writer's flush completes the line.
    with open(w.index_path, "a") as f:
        f.write(last[cut:] + "\n")
    added = r.refresh()
    total_after = len(r)
    last_ok, why = check(r.frame(10), 10)
    r.close()

    print(f"  with a half-written line: {n_partial_seen} frames "
          f"(expected 9), flagged partial={partial_flagged}")
    print(f"  the 9 complete rows read correctly: {frag_ok}")
    print(f"  after the line completes: +{added} -> {total_after} frames, "
          f"frame 10 correct: {last_ok} {why}")
    ok = (n_partial_seen == 9 and partial_flagged >= 1 and frag_ok
          and added == 1 and total_after == 10 and last_ok)
    print(f"  -> {'OK' if ok else 'MISMATCH'}")
    return ok


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--frames", type=int, default=600)
    p.add_argument("--readers", type=int, default=4)
    p.add_argument("--quality", type=int, default=85)
    p.add_argument("--keep", action="store_true")
    args = p.parse_args()

    tmp = tempfile.mkdtemp(prefix="avistress-")
    failures = []
    try:
        print(f"[1] {args.readers} random readers + 1 tail reader against a "
              f"live writer ({args.frames} frames)")
        stats, tail_out, path = run_live(tmp, args.frames, args.readers,
                                        args.quality)
        if stats.errors:
            failures.append("concurrent read")
        if tail_out.get("seen") != args.frames:
            failures.append("tail completeness")

        print(f"\n[2] finished file is a standard AVI")
        if not validate_final(path, args.frames):
            failures.append("final container")

        print(f"\n[3] writer killed before close(), then repaired")
        if not run_crash(tmp, min(args.frames, 200), args.quality):
            failures.append("crash recovery")

        print(f"\n[4] a torn index line, forced")
        if not run_torn(tmp, args.quality):
            failures.append("torn index line")
    finally:
        if args.keep:
            print(f"\nkept {tmp}")
        else:
            shutil.rmtree(tmp, ignore_errors=True)

    print()
    if failures:
        print("FAIL: " + ", ".join(failures))
        return 1
    print("PASS: all checks")
    return 0


if __name__ == "__main__":
    sys.exit(main())

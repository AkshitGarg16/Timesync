"""Append-only MJPEG AVI that is safe to read while it is still being written.

`cv2.VideoWriter` cannot be used for this: it buffers the whole stream and
writes nothing until `release()`, so a concurrent reader cannot even open the
file. On top of that, AVI keeps its frame index (`idx1`) and its header size
and frame-count fields at the *end* of the file, filled in at close, so even a
flushed AVI cannot be seeked by a normal reader until recording stops.

`MjpegAviWriter` solves both: it writes the container itself, flushing every
frame, and records each frame's byte offset in a sidecar index CSV.
`MjpegAviReader` then random-accesses any frame through that index without
touching `idx1`, so it works on a file that is still growing.

The write order is the whole protocol, and it matters:

    1. append the JPEG chunk to the .avi  and flush
    2. append the row to the index        and flush

An index row is therefore only visible once the bytes it points at are
already on disk. A reader that sees row N can always read frame N. Doing it
in the other order would hand readers offsets into data that does not exist
yet. Readers must also ignore a trailing partial line, since a reader can
look between the writer's two flushes; `MjpegAviReader` does.

On `close()` the index is written as a real `idx1` chunk and the header
fields are patched, so the finished file is a standard AVI that ffmpeg, VLC
and `cv2.VideoCapture` all read normally. If the process is killed first,
every frame is still in the file and on disk, the sidecar index still
describes them all, and `repair()` rebuilds the header and `idx1` from it.
"""
import os
import struct

AVIF_HASINDEX = 0x10
AVIIF_KEYFRAME = 0x10
CHUNK_ID = b"00dc"
INDEX_COLUMNS = ("frame", "t_ns", "avi_pos", "offset", "length")

# Byte offsets of the fields that can only be known at close. Asserted
# against the real header in _build_header, so a layout edit cannot
# silently desynchronise them.
RIFF_SIZE_FIELD = 4
TOTAL_FRAMES_FIELD = 48
STRH_LENGTH_FIELD = 140
MOVI_SIZE_FIELD = 216
MOVI_FOURCC_POS = 220
DATA_START = 224


def _build_header(width, height, fps):
    avih = struct.pack(
        "<10I16s",
        int(round(1e6 / fps)) if fps else 33333,  # dwMicroSecPerFrame
        0,                                       # dwMaxBytesPerSec
        0,                                       # dwPaddingGranularity
        AVIF_HASINDEX,                           # dwFlags
        0,                                       # dwTotalFrames  (patched)
        0,                                       # dwInitialFrames
        1,                                       # dwStreams
        0,                                       # dwSuggestedBufferSize
        width, height,
        b"\0" * 16,                              # dwReserved[4]
    )
    assert len(avih) == 56, len(avih)

    strh = struct.pack(
        "<4s4sI2H8I4h",
        b"vids", b"MJPG",
        0,                      # dwFlags
        0, 0,                   # wPriority, wLanguage
        0,                      # dwInitialFrames
        1000,                   # dwScale
        int(round(fps * 1000)) if fps else 30000,  # dwRate
        0,                      # dwStart
        0,                      # dwLength  (patched)
        0,                      # dwSuggestedBufferSize
        0,                      # dwQuality
        0,                      # dwSampleSize
        0, 0, width, height,    # rcFrame
    )
    assert len(strh) == 56, len(strh)

    strf = struct.pack("<I2i2H2i2I2I", 40, width, height, 1, 24,
                       int.from_bytes(b"MJPG", "little"), width * height * 3,
                       0, 0, 0, 0)
    assert len(strf) == 40, len(strf)

    strl = b"strl" + b"strh" + struct.pack("<I", 56) + strh \
                   + b"strf" + struct.pack("<I", 40) + strf
    hdrl = b"hdrl" + b"avih" + struct.pack("<I", 56) + avih \
                   + b"LIST" + struct.pack("<I", len(strl)) + strl

    head = (b"RIFF" + struct.pack("<I", 0) + b"AVI "
            + b"LIST" + struct.pack("<I", len(hdrl)) + hdrl
            + b"LIST" + struct.pack("<I", 0) + b"movi")

    # The patch offsets above are load-bearing; prove them against reality.
    assert head[0:4] == b"RIFF"
    assert head[TOTAL_FRAMES_FIELD - 4:TOTAL_FRAMES_FIELD] == struct.pack("<I", AVIF_HASINDEX)
    assert head[STRH_LENGTH_FIELD - 32:STRH_LENGTH_FIELD - 28] == b"vids"
    assert head[MOVI_FOURCC_POS:MOVI_FOURCC_POS + 4] == b"movi"
    assert len(head) == DATA_START, (len(head), DATA_START)
    return head


class MjpegAviWriter:
    """Single-writer MJPEG AVI with a flushed sidecar index."""

    def __init__(self, path, width, height, fps=30.0, index_path=None,
                 fsync_interval_s=2.0):
        self.path = path
        self.index_path = (index_path if index_path is not None
                           else os.path.splitext(path)[0] + "_index.csv")
        self.width = width
        self.height = height
        self.fps = fps
        self.frames = 0
        self.bytes_written = 0
        self._entries = []
        self._closed = False
        self._fsync_interval = fsync_interval_s

        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        self._f = open(path, "xb")
        self._f.write(_build_header(width, height, fps))
        self._f.flush()

        self._idx = open(self.index_path, "x", newline="")
        self._idx.write(",".join(INDEX_COLUMNS) + "\n")
        self._idx.flush()

        import time
        self._time = time.monotonic
        self._last_fsync = self._time()

    def write(self, jpeg: bytes, frame: int, t_ns: int) -> int:
        """Append one already-encoded JPEG. Returns its position in the AVI."""
        if self._closed:
            raise ValueError("writer is closed")
        if not (jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9"):
            raise ValueError("not a complete JPEG (bad SOI/EOI)")

        chunk_pos = self._f.tell()
        payload_off = chunk_pos + 8
        pad = len(jpeg) & 1
        self._f.write(CHUNK_ID + struct.pack("<I", len(jpeg)) + jpeg
                      + (b"\0" if pad else b""))
        # Barrier: the bytes must be on disk before the index advertises them.
        self._f.flush()

        pos = self.frames
        self._entries.append((chunk_pos - MOVI_FOURCC_POS, len(jpeg)))
        self.frames += 1
        self.bytes_written = self._f.tell()

        self._idx.write(f"{frame},{t_ns},{pos},{payload_off},{len(jpeg)}\n")
        self._idx.flush()

        now = self._time()
        if now - self._last_fsync >= self._fsync_interval:
            os.fsync(self._f.fileno())
            os.fsync(self._idx.fileno())
            self._last_fsync = now
        return pos

    def close(self):
        """Append idx1, patch the header, and leave a standard AVI behind."""
        if self._closed:
            return
        self._closed = True
        try:
            movi_end = self._f.tell()
            idx = bytearray()
            for off, size in self._entries:
                idx += CHUNK_ID + struct.pack("<III", AVIIF_KEYFRAME, off, size)
            self._f.write(b"idx1" + struct.pack("<I", len(idx)) + bytes(idx))
            end = self._f.tell()

            self._f.seek(RIFF_SIZE_FIELD)
            self._f.write(struct.pack("<I", end - 8))
            self._f.seek(TOTAL_FRAMES_FIELD)
            self._f.write(struct.pack("<I", self.frames))
            self._f.seek(STRH_LENGTH_FIELD)
            self._f.write(struct.pack("<I", self.frames))
            self._f.seek(MOVI_SIZE_FIELD)
            self._f.write(struct.pack("<I", movi_end - MOVI_FOURCC_POS))
            self._f.flush()
            os.fsync(self._f.fileno())
        finally:
            self._f.close()
            try:
                self._idx.flush()
                os.fsync(self._idx.fileno())
            finally:
                self._idx.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class MjpegAviReader:
    """Random access into an MJPEG AVI through its sidecar index.

    Safe against a writer appending concurrently: it never reads `idx1` or
    the header counters, and `refresh()` picks up frames added since the last
    call. A partially written trailing index line is ignored until complete.

    One hazard worth stating, because it is easy to get wrong: learning that
    the writer has stopped does NOT mean you have seen every frame. Rows
    flushed after your last `refresh()` are still waiting. Always refresh
    once more after the writer finishes, and only conclude the stream has
    ended when a refresh returns 0:

        while True:
            added = reader.refresh()
            ... consume newly available frames ...
            if writer_finished and added == 0:
                break
    """

    def __init__(self, path, index_path=None):
        self.path = path
        self.index_path = (index_path if index_path is not None
                           else os.path.splitext(path)[0] + "_index.csv")
        self._f = open(path, "rb")
        self._rows = []
        self._by_frame = {}
        self._idx_pos = 0
        self._header_skipped = False
        self.partial_reads = 0
        self.refresh()

    def refresh(self) -> int:
        """Pull in any index rows that have appeared. Returns how many."""
        added = 0
        with open(self.index_path, "r", newline="") as f:
            f.seek(self._idx_pos)
            data = f.read()
            # Only whole lines are trustworthy; a reader can land between the
            # writer's video flush and its index flush.
            cut = data.rfind("\n")
            if cut == -1:
                if data:
                    self.partial_reads += 1
                return 0
            if len(data) - cut > 1:
                self.partial_reads += 1
            self._idx_pos += cut + 1
            for line in data[:cut].split("\n"):
                if not line:
                    continue
                if not self._header_skipped:
                    self._header_skipped = True
                    if line.startswith("frame,"):
                        continue
                parts = line.split(",")
                if len(parts) != len(INDEX_COLUMNS):
                    self.partial_reads += 1
                    continue
                try:
                    frame, t_ns, pos, off, length = (int(x) for x in parts)
                except ValueError:
                    self.partial_reads += 1
                    continue
                row = {"frame": frame, "t_ns": t_ns, "avi_pos": pos,
                       "offset": off, "length": length}
                self._by_frame[frame] = row
                self._rows.append(row)
                added += 1
        return added

    def __len__(self):
        return len(self._rows)

    @property
    def frames(self):
        return [r["frame"] for r in self._rows]

    def jpeg(self, frame: int) -> bytes:
        """Raw JPEG bytes for a camera frame number."""
        row = self._by_frame.get(frame)
        if row is None:
            raise KeyError(f"frame {frame} not in index")
        self._f.seek(row["offset"])
        b = self._f.read(row["length"])
        if len(b) != row["length"]:
            raise IOError(f"frame {frame}: short read "
                          f"({len(b)} of {row['length']} bytes)")
        if not (b[:2] == b"\xff\xd8" and b[-2:] == b"\xff\xd9"):
            raise IOError(f"frame {frame}: JPEG markers missing")
        return b

    def frame(self, frame: int):
        """Decoded BGR image for a camera frame number."""
        import cv2
        import numpy as np
        img = cv2.imdecode(np.frombuffer(self.jpeg(frame), np.uint8),
                           cv2.IMREAD_COLOR)
        if img is None:
            raise IOError(f"frame {frame}: JPEG failed to decode")
        return img

    def close(self):
        self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def repair(path, index_path=None, fps=30.0):
    """Rebuild idx1 and the header of an AVI left unfinalised by a crash."""
    index_path = (index_path if index_path is not None
                  else os.path.splitext(path)[0] + "_index.csv")
    rows = []
    with open(index_path, "r", newline="") as f:
        data = f.read()
    cut = data.rfind("\n")
    for line in data[:cut if cut != -1 else 0].split("\n"):
        if not line or line.startswith("frame,"):
            continue
        parts = line.split(",")
        if len(parts) != len(INDEX_COLUMNS):
            continue
        try:
            rows.append(tuple(int(x) for x in parts))
        except ValueError:
            continue
    if not rows:
        raise ValueError(f"{index_path}: no complete index rows")

    with open(path, "r+b") as f:
        f.seek(0, os.SEEK_END)
        size = f.tell()
        last_off, last_len = rows[-1][3], rows[-1][4]
        movi_end = last_off + last_len + (last_len & 1)
        if movi_end > size:
            raise ValueError(f"{path}: index points past end of file "
                             f"({movi_end} > {size})")
        f.seek(movi_end)
        f.truncate()
        idx = bytearray()
        for _, _, _, off, length in rows:
            idx += CHUNK_ID + struct.pack("<III", AVIIF_KEYFRAME,
                                          off - 8 - MOVI_FOURCC_POS, length)
        f.write(b"idx1" + struct.pack("<I", len(idx)) + bytes(idx))
        end = f.tell()
        f.seek(RIFF_SIZE_FIELD); f.write(struct.pack("<I", end - 8))
        f.seek(TOTAL_FRAMES_FIELD); f.write(struct.pack("<I", len(rows)))
        f.seek(STRH_LENGTH_FIELD); f.write(struct.pack("<I", len(rows)))
        f.seek(MOVI_SIZE_FIELD); f.write(struct.pack("<I", movi_end - MOVI_FOURCC_POS))
        f.flush()
        os.fsync(f.fileno())
    return len(rows)

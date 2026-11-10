# timesync

Records a camera, an ArduPilot flight controller and a LightWare SF45/B
scanning LiDAR into one time-aligned CSV. Every sample is put on a common
host-monotonic timeline first, then each camera frame becomes one row of
`frames.csv` with the other streams sampled at that frame's timestamp.

Pure Python, no ROS. Each device is a thread; the main loop only aligns and
writes.

```
sources/*  ──►  Bus (queue of Records)  ──►  FrameAligner  ──►  frames.csv
   │                                              ▲
   └── per-source raw CSVs (fcu_*.csv, lidar_points.csv)
```

## Install

```bash
pip install pymavlink pyserial opencv-python numpy
```

Python 3.10. The user must be in `dialout` (or equivalent) for the serial
ports.

## Run

```bash
cd ~/timesync

# everything, FCU over MAVProxy's UDP out
mavproxy.py --master=/dev/serial/by-id/usb-CubePilot_CubeOrange+_...-if00 \
            --baudrate 921600 --out=udp:127.0.0.1:14550 --streamrate=-1
python3 main.py --camera /dev/video0 --fcu udpin:0.0.0.0:14550 --sf45 /dev/ttyUSB0

# FCU only, direct serial (nothing else can share the port then)
python3 main.py --fcu /dev/ttyACM0 --baud 921600

# LiDAR only
python3 main.py --sf45 /dev/ttyUSB0 --sf45-rate 6 --low -45 --high 45
```

Ctrl-C stops; sources are torn down, pending frames flushed, files fsynced.
A status line goes to stderr every `--status-s` seconds with row counts,
per-stream publish counts, queue depth, drops and any source that failed.

At least one of `--camera/--fcu/--sf45` is required. Without `--camera`
there is no trigger stream, so `frames.csv` stays empty (header only) while
the per-source CSVs still fill — that is the normal way to log one device.

### Flags

| flag | default | meaning |
| --- | --- | --- |
| `--out` | `runs` | parent dir; each run gets `runs/<UTC timestamp>/` |
| `--camera` | – | video device index or path, e.g. `0` or `/dev/video0` |
| `--fcu` | – | pymavlink connection string or serial port |
| `--baud` | 921600 | FCU baud (serial only) |
| `--sf45` | – | SF45/B serial port |
| `--sf45-baud` | 115200 | SF45 baud |
| `--sf45-rate` | 6 | update-rate setting 1–7 (3≈200, 4≈400, 5–7≈478 points/s) |
| `--low` / `--high` | device's | scan limits in degrees (−160..−10 / 10..160) |
| `--sectors` | 12 | sweep-summary sectors; edges land in `lidar_info.json` |
| `--max-wait-ms` | 600 | hard cap on how long a frame waits for slow streams |
| `--cam-offset-ms` | 0 | added to each frame stamp before lookup |
| `--status-s` | 5 | status line interval |

## How the timeline is built

`clock.py` holds the pieces:

- **`Clock`** — host timeline is `CLOCK_MONOTONIC`. A realtime offset is
  sampled once (sandwiched between two monotonic reads) so rows can carry a
  Unix time as well; `capture_uncertainty_ns` in the manifest is that read's
  error bound.
- **`ClockOffset`** — per-device offset/skew filter. Round trips feed
  `observe_roundtrip` (RTT over `max_rtt_ns` rejected); a receive-only link
  falls back to `observe_oneway`, which includes link latency. Gains start
  loose and tighten over `convergence_window` samples; before convergence a
  device stamp is not trusted and arrival time is used instead. A run of
  large deviations (e.g. an autopilot reboot) resets the filter.
- **`Uint32Unwrapper`** — turns ArduPilot's 32-bit `time_boot_ms` into a
  monotonic nanosecond count across wraps.

Per source:

- **FCU** (`sources/mavlink.py`) — TIMESYNC round trips at 10 Hz (50 Hz until
  converged) map `time_boot_ms` (+0.5 ms, since it is `millis()` truncated)
  onto host time, clamped to never exceed arrival. Requests ATTITUDE at 50 Hz
  and GLOBAL_POSITION_INT at 10 Hz via `SET_MESSAGE_INTERVAL`, and
  re-requests if a stream drops below half rate. Measured RTT on the
  connected CubeOrange+ through MAVProxy: 2.7–8.3 ms, median 4.8 ms.
- **LiDAR** (`sources/lidar_driver.py`) — a point's measurement time is
  `arrival − 4.7 ms link latency − 0.9 ms angle lag`, and its angle is the
  yaw series interpolated at that time (never derived from time, the head
  slows at the arc ends). Points are grouped into sweeps split at yaw
  reversal. Device settings are changed in RAM only and restored on stop; a
  watchdog reconfigures the unit if it resets mid-run.

Measured per-device behaviour — packet formats, rates, the first-return
freeze, which command ids reboot the SF45 — is written up in each driver's
module docstring. Read those before changing a driver.

## Alignment

`aligner.py` keeps a short history per stream and samples it at each frame
stamp. Mode per stream, with the max gap beyond which the row is left blank:

| stream | mode | max gap | notes |
| --- | --- | --- | --- |
| `fcu.att` | interp | 200 ms | linear, yaw wrapped in radians |
| `fcu.gpos` | interp | 500 ms | linear, heading wrapped in degrees |
| `lidar.sweep` | event | 1500 ms | last sweep at or before the frame |
| `lidar.pt` | nearest | 50 ms | closest point either side |

Each stream contributes an `age_ms` column (`dt_ms`, signed, for nearest) so
a row's staleness is visible in the data. A frame is emitted once every
stream has covered its timestamp, or after `--max-wait-ms`; rows released on
the timeout are counted as `timed_out` in the status line. A stream is only
written off as quiet when its newest data *and* its last arrival are both
older than its max gap — a stream that delivers with a fixed delay (the
LiDAR holds points ~125 ms) is waited for, not skipped.

Out-of-order samples are inserted in place (counted as `late`). The bus drops
oldest-first when its 20 000-record queue fills, per stream, reported in the
status line.

## Output

`runs/<UTC timestamp>/`

| file | contents |
| --- | --- |
| `manifest.json` | boot times, clock uncertainty, full args, column list |
| `frames.csv` | one row per frame: `frame, t_ns, t_unix, t_capture_ns` + every stream's columns |
| `fcu_att.csv`, `fcu_gpos.csv` | every message with host, Unix, arrival and raw device time |
| `fcu_timesync.csv` | every round trip: RTT, accepted, offset, skew, converged — enough to refit the mapping offline |
| `fcu_info.json` | link, autopilot, requested rates, sync mode and counters |
| `lidar_points.csv` | every point: times, sweep, angle, raw yaw, distance, x/y, first/last return, strength, temp |
| `lidar_info.json` | device identity, settings, angle convention, sector edges, timestamp model, run stats |
| `video/video.avi` | the video feed: one continuous MJPEG AVI, readable while it grows |
| `video/video_index.csv` | `frame,t_ns,avi_pos,offset,length` — frame number to byte offset |
| `cam_info.json` | negotiated format, measured fps, stamp model, grab/decode/write failure counts |

`t_ns` is host monotonic, `t_capture_ns` the frame stamp before
`--cam-offset-ms`, `t_unix` seconds derived from the manifest's offset.
LiDAR angles: 0 straight ahead, positive left; `x_m` forward, `y_m` left.

`runs/20260930T100700Z/` is a real capture kept as a reference for the
column layout and for what healthy numbers look like.

## Camera

`sources/camera.py` is a standalone driver for the NeoHD PTZ camera — VISCA
control over serial (`sources/visca.py`) plus OpenCV video — and can also
serve video and control over TCP:

```bash
python3 -m sources.camera serve --serial /dev/ttyACM0 --video 0
python3 -m sources.camera view <host>          # z/x zoom, f/v focus, q quit
```

(Run it as a module, not `python3 sources/camera.py`; it imports `visca`
relatively.)

The same module also exposes `CameraSource`, the `Source` subclass `main.py`
loads for `--camera`. It is the aligner's **trigger**: one frame published
here is one row in `frames.csv`, so without `--camera` the file stays empty
no matter how healthy the other streams are.

It takes a device index or path (`--camera 0`, `--camera /dev/video0`) and
opens video only — VISCA control is not needed to record, so no serial port
is touched unless you pass `visca_port`, which routes capture through
`NeoHDCamera` so you can drive the lens while recording.

Each frame is stamped on `CLOCK_MONOTONIC` between `grab()` and `retrieve()`,
so the stamp is the buffer dequeue rather than the decode that follows it;
`latency_ms` (or `--cam-offset-ms`) shifts it to compensate for exposure and
transfer delay. Frames are appended to one continuous `video/video.avi` from
a writer thread, so encoding and disk I/O never delay the next grab, and the
`frame` column is the key into `video_index.csv`. If that thread falls
behind, frames are dropped from the video (counted in `dropped_writes`) and
never from the bus — a gap in the imagery is recoverable, a gap in the
timeline is not. `cam_info.json` records the negotiated format, measured fps
and the failure counters.

Note the capture queue is deliberately left at the driver default: forcing
`CAP_PROP_BUFFERSIZE=1` costs exactly half the frame rate, because after
`grab()` the driver has no spare buffer to fill while the frame is decoded.

## Video, and reading a run while it records

The video feed is always one continuous `video/video.avi`, with a sidecar
`video/video_index.csv`. There is no per-frame-file mode. `follow.py` reads a
run live, joining each `frames.csv` row to its video frame:

```bash
python3 follow.py                           # newest run under runs/, verify only
python3 follow.py --show                    # live window, data drawn on the frame
python3 follow.py --save annotated.avi      # one AVI with the data burnt in
```

The frame number is burned into the top-left of each image (`--no-stamp` to
turn it off), so the correspondence to `frames.csv` is checkable by eye as
well as through the index.

### Size, and why MJPEG

MJPEG is bulky, and `--quality` is the knob (measured at 640x480, 30 fps):

| `--quality` | per frame | rate | per hour |
| --- | --- | --- | --- |
| 90 (default) | 44.8 KB | 1.38 MB/s | 4.95 GB |
| 75 | 25.8 KB | 0.79 MB/s | 2.86 GB |
| 60 | 20.2 KB | 0.62 MB/s | 2.24 GB |

An inter-frame codec (H.264, XVID) would be far smaller, but it is
incompatible with reading a run while it records, which is the point of this
layout. Those codecs encode most frames as differences from their
neighbours, so no frame can be decoded from its own bytes alone, there is no
per-frame byte range to index, and the encoder must buffer across frames
before emitting anything. MJPEG is all-intra: every frame is independent, so
every frame has an offset and a length, and a reader can decode frame N from
the bytes already on disk. The redundancy is what buys the random access.

If a run is long enough that size wins over live access, encode after the
fact and keep the AVI as the working copy:

```bash
ffmpeg -i runs/<ts>/video/video.avi -c:v libx264 -crf 23 archive.mp4
```

### Why not `cv2.VideoWriter`

It cannot be read while recording — and not marginally so. Measured here: a
`VideoWriter` holding 30 frames had written **0 bytes**; the whole stream is
buffered until `release()`, and a concurrent reader cannot even open the
file. AVI compounds this by keeping its frame index (`idx1`) and its header
size and frame-count fields at the *end* of the file, written at close.

So `avi.py` writes the container itself. Its protocol is two ordered steps
per frame, and the order is the whole point:

1. append the JPEG chunk to the `.avi`, **flush**
2. append the row to the index, **flush**

An index row therefore only becomes visible once the bytes it points at are
already on disk, so a reader that sees row N can always read frame N.
Reversing the order would hand readers offsets into data that does not exist
yet. `MjpegAviReader` random-accesses frames through that index and never
touches `idx1`, which is what makes a growing file seekable.

Two hazards worth knowing, both handled and both tested:

* **Torn index lines.** A reader can land between the two flushes, so only
  whole lines are parsed; a trailing fragment is ignored until complete.
* **Deciding the stream has ended.** Learning that the writer stopped does
  *not* mean you have seen every frame — rows flushed after your last
  `refresh()` are still waiting. Refresh again after the writer finishes and
  only stop when a refresh returns 0. Getting this wrong is not theoretical:
  it is what made the stress test first report 287 of 600 frames.

On `close()` the index is written as a real `idx1` chunk and the header
fields are patched, so the result is a standard AVI — verified with `ffprobe`
and `cv2.VideoCapture`. If the process is killed first, every frame is still
on disk and `avi.repair()` rebuilds the header and `idx1` from the sidecar
index.

Dropping stays honest: if the writer queue overflows, the frame is lost from
the imagery but never from the timeline, and because the index stores real
frame numbers a drop leaves a gap in the numbering instead of silently
shifting every later frame.

### Testing it

```bash
python3 test_concurrent.py          # ~10 s
```

Four checks: four random readers plus an in-order tail reader against a live
writer; the finished file against `ffprobe` and `cv2`; a writer killed before
`close()` then repaired; and a deliberately half-written index line. Frames
are filled with a colour derived from their own frame number and verified
after decoding, so a torn read, a stale index row or an off-by-one offset
shows up as wrong content rather than passing silently.

## Adding a source

Subclass `sources.base.Source` (a daemon thread with `setup/poll/teardown`
and error counting that keeps the thread alive), stamp samples on the host
timeline, and `bus.publish(Record(...))`. `t_host_ns` is the sample's own
time; `t_arrival_ns` tells the aligner how far the stream is covered. Then
add a `StreamSpec` in `main.py:stream_specs` and a branch in `build_sources`.

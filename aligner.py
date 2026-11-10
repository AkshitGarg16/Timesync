import bisect
import math
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

INTERP, EVENT, NEAREST = "interp", "event", "nearest"


def hold(a, b, w):
    return a


def lerp(a, b, w):
    if a is None or b is None:
        return a if b is None else b
    return a + (b - a) * w


def angle_lerp_deg(a, b, w):
    if a is None or b is None:
        return a if b is None else b
    d = ((b - a + 180.0) % 360.0) - 180.0
    return (a + d * w) % 360.0


def angle_lerp_rad(a, b, w):
    if a is None or b is None:
        return a if b is None else b
    d = ((b - a + math.pi) % (2 * math.pi)) - math.pi
    return (a + d * w + math.pi) % (2 * math.pi) - math.pi


class StreamSpec:

    def __init__(self, name: str, columns: List[str], mode: str = INTERP,
                 interp: Dict[str, Callable] = None,
                 max_gap_ns: int = 500_000_000, history: int = 4096):
        if mode not in (INTERP, EVENT, NEAREST):
            raise ValueError(f"unknown mode {mode!r}")
        self.name = name
        self.columns = list(columns)
        self.mode = mode
        self.interp = interp or {}
        self.max_gap_ns = max_gap_ns
        self.history = history


class StreamBuffer:

    def __init__(self, spec: StreamSpec):
        self.spec = spec
        self._t: Deque[int] = deque(maxlen=spec.history)
        self._v: Deque[Dict[str, Any]] = deque(maxlen=spec.history)
        self.late = 0

    def add(self, t_ns: int, fields: Dict[str, Any]):
        if self._t and t_ns < self._t[-1]:
            i = bisect.bisect_right(self._t, t_ns)
            if len(self._t) == self._t.maxlen:
                if i == 0:
                    return
                self._t.popleft(); self._v.popleft()
                i -= 1
            self._t.insert(i, t_ns)
            self._v.insert(i, fields)
            self.late += 1
        else:
            self._t.append(t_ns)
            self._v.append(fields)

    def newest_ns(self) -> Optional[int]:
        return self._t[-1] if self._t else None

    def prune_before(self, t_ns: int):
        while len(self._t) > 2 and self._t[1] < t_ns:
            self._t.popleft()
            self._v.popleft()

    def sample_at(self, t_ns: int) -> Tuple[Dict[str, Any], Optional[int]]:
        if not self._t:
            return {}, None
        mode = self.spec.mode
        if mode == EVENT:
            return self._event(t_ns)
        if mode == NEAREST:
            return self._nearest(t_ns)
        return self._interp(t_ns)

    def _event(self, t_ns):
        i = bisect.bisect_right(self._t, t_ns) - 1
        if i < 0:
            return {}, None
        age = t_ns - self._t[i]
        if age > self.spec.max_gap_ns:
            return {}, age
        return dict(self._v[i]), age

    def _nearest(self, t_ns):
        i = bisect.bisect_left(self._t, t_ns)
        cands = [j for j in (i - 1, i) if 0 <= j < len(self._t)]
        j = min(cands, key=lambda k: abs(self._t[k] - t_ns))
        dt = self._t[j] - t_ns
        if abs(dt) > self.spec.max_gap_ns:
            return {}, dt
        return dict(self._v[j]), dt

    def _interp(self, t_ns):
        i = bisect.bisect_left(self._t, t_ns)
        if i < len(self._t) and self._t[i] == t_ns:
            return dict(self._v[i]), 0
        if i == 0 or i == len(self._t):
            j = 0 if i == 0 else -1
            gap = abs(self._t[j] - t_ns)
            return ({}, gap) if gap > self.spec.max_gap_ns else (dict(self._v[j]), gap)
        t0, t1 = self._t[i - 1], self._t[i]
        if t1 - t0 > self.spec.max_gap_ns:
            return {}, t_ns - t0
        w = (t_ns - t0) / (t1 - t0)
        v0, v1 = self._v[i - 1], self._v[i]
        out = {}
        for k in v0.keys() | v1.keys():
            fn = self.spec.interp.get(k, hold)
            out[k] = fn(v0.get(k), v1.get(k), w)
        return out, 0


class FrameAligner:

    def __init__(self, specs: List[StreamSpec], trigger: str = "cam",
                 trigger_columns: List[str] = ("frame",),
                 max_wait_ns: int = 600_000_000, frame_offset_ns: int = 0,
                 to_unix_ns: Callable[[int], int] = None):
        self.trigger = trigger
        self.max_wait_ns = max_wait_ns
        self.frame_offset_ns = frame_offset_ns
        self.to_unix_ns = to_unix_ns
        self.buffers: Dict[str, StreamBuffer] = {
            sp.name: StreamBuffer(sp) for sp in specs if sp.name != trigger}
        self._covered: Dict[str, int] = {}
        self._seen: Dict[str, int] = {}
        self._pending: List[Tuple[int, int, int, Dict[str, Any]]] = []
        self._seq = 0
        self.rows_emitted = 0
        self.rows_timed_out = 0

        self._trigger_cols = [c for c in trigger_columns if c != "frame"]
        self._columns = ["frame", "t_ns", "t_unix", "t_capture_ns"]
        self._columns += [f"{trigger}.{c}" for c in self._trigger_cols]
        for name, buf in self.buffers.items():
            self._columns += [f"{name}.{c}" for c in buf.spec.columns]
            self._columns.append(f"{name}.dt_ms" if buf.spec.mode == NEAREST
                                 else f"{name}.age_ms")

    def columns(self) -> List[str]:
        return list(self._columns)

    @property
    def pending(self) -> int:
        return len(self._pending)

    def add(self, rec) -> None:
        if rec.stream == self.trigger:
            t = rec.t_host_ns + self.frame_offset_ns
            self._seq += 1
            bisect.insort(self._pending, (t, self._seq, rec.t_host_ns, rec.fields))
            return
        buf = self.buffers.get(rec.stream)
        if buf is None:
            return
        buf.add(rec.t_host_ns, rec.fields)
        arrived = rec.t_arrival_ns or rec.t_host_ns
        if arrived > self._seen.get(rec.stream, -1):
            self._seen[rec.stream] = arrived
        mark = rec.t_host_ns
        if buf.spec.mode == EVENT and rec.t_arrival_ns:
            mark = max(mark, rec.t_arrival_ns)
        if mark > self._covered.get(rec.stream, -1):
            self._covered[rec.stream] = mark

    def _ready(self, t_frame: int, now_ns: int) -> bool:
        return (now_ns - t_frame >= self.max_wait_ns
                or not self._waiting_on(t_frame, now_ns))

    def _waiting_on(self, t_frame: int, now_ns: int) -> bool:
        # A stream is only given up on as quiet when its data is older than
        # max_gap AND nothing has arrived for max_gap. A stream that is
        # delivering, but with a fixed delay (e.g. the LiDAR holds points
        # ~125 ms), is waited for instead of being skipped.
        for name, buf in self.buffers.items():
            covered = self._covered.get(name)
            if covered is None:
                continue
            if covered >= t_frame:
                continue
            if (t_frame - covered > buf.spec.max_gap_ns
                    and now_ns - self._seen.get(name, now_ns) > buf.spec.max_gap_ns):
                continue
            return True
        return False

    def drain(self, now_ns: int) -> List[Dict[str, Any]]:
        rows = []
        while self._pending and self._ready(self._pending[0][0], now_ns):
            t, _, t_cap, fields = self._pending[0]
            if self._waiting_on(t, now_ns):
                self.rows_timed_out += 1
            self._pending.pop(0)
            rows.append(self._build(t, t_cap, fields))
        return rows

    def finalize(self) -> List[Dict[str, Any]]:
        rows = [self._build(t, t_cap, f) for t, _, t_cap, f in self._pending]
        self._pending.clear()
        return rows

    def _build(self, t: int, t_cap: int, fields: Dict[str, Any]) -> Dict[str, Any]:
        row: Dict[str, Any] = {c: None for c in self._columns}
        row["frame"] = fields.get("frame")
        row["t_ns"] = t
        row["t_capture_ns"] = t_cap
        if self.to_unix_ns is not None:
            row["t_unix"] = f"{self.to_unix_ns(t) / 1e9:.6f}"
        for c in self._trigger_cols:
            row[f"{self.trigger}.{c}"] = fields.get(c)
        for name, buf in self.buffers.items():
            vals, age = buf.sample_at(t)
            for k, v in vals.items():
                key = f"{name}.{k}"
                if key in row:
                    row[key] = v
            col = "dt_ms" if buf.spec.mode == NEAREST else "age_ms"
            row[f"{name}.{col}"] = None if age is None else round(age / 1e6, 3)
            buf.prune_before(t)
        self.rows_emitted += 1
        return row

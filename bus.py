import queue
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class Record:
    stream: str
    t_host_ns: int
    t_arrival_ns: int
    fields: Dict[str, Any]
    t_device_ns: Optional[int] = None
    seq: int = 0


class Bus:

    def __init__(self, maxsize: int = 20000):
        self._q: queue.Queue = queue.Queue(maxsize=maxsize)
        self._lock = threading.Lock()
        self.dropped: Dict[str, int] = {}
        self.published: Dict[str, int] = {}

    def publish(self, rec: Record):
        with self._lock:
            self.published[rec.stream] = self.published.get(rec.stream, 0) + 1
        try:
            self._q.put_nowait(rec)
        except queue.Full:
            try:
                self._q.get_nowait()
                self._q.put_nowait(rec)
            except (queue.Empty, queue.Full):
                pass
            with self._lock:
                self.dropped[rec.stream] = self.dropped.get(rec.stream, 0) + 1

    def get(self, timeout: float = 0.1) -> Optional[Record]:
        try:
            return self._q.get(timeout=timeout)
        except queue.Empty:
            return None

    def depth(self) -> int:
        return self._q.qsize()

    def stats(self) -> Dict[str, Dict[str, int]]:
        with self._lock:
            return {"published": dict(self.published), "dropped": dict(self.dropped),
                    "depth": self._q.qsize()}

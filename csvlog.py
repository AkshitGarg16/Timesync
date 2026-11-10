import csv
import os
import time
from typing import Any, Dict, List


class CsvLog:

    def __init__(self, path: str, columns: List[str],
                 fsync_interval_s: float = 2.0):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self._f = open(path, "x", newline="")
        self.path = path
        self.columns = list(columns)
        self._w = csv.DictWriter(self._f, fieldnames=self.columns,
                                 extrasaction="ignore", restval="")
        self._w.writeheader()
        self._f.flush()
        self._fsync_interval = fsync_interval_s
        self._last_fsync = time.monotonic()
        self.rows = 0

    def write(self, row: Dict[str, Any]):
        self._w.writerow(row)
        self.rows += 1
        self._f.flush()
        now = time.monotonic()
        if now - self._last_fsync >= self._fsync_interval:
            os.fsync(self._f.fileno())
            self._last_fsync = now

    def write_many(self, rows: List[Dict[str, Any]]):
        for row in rows:
            self.write(row)

    def close(self):
        if self._f.closed:
            return
        try:
            self._f.flush()
            os.fsync(self._f.fileno())
        finally:
            self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

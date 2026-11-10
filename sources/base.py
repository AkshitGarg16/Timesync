import sys
import threading
import traceback


class Source(threading.Thread):

    def __init__(self, name: str, bus, clock):
        super().__init__(name=name, daemon=True)
        self.bus = bus
        self.clock = clock
        self._stop_evt = threading.Event()
        self.errors = 0
        self.last_error = None
        self.failed = False

    def setup(self):
        pass

    def poll(self):
        raise NotImplementedError

    def teardown(self):
        pass

    def stop(self):
        self._stop_evt.set()

    @property
    def stopping(self) -> bool:
        return self._stop_evt.is_set()

    def _report(self, where: str):
        self.errors += 1
        err = traceback.format_exc().strip().splitlines()[-1]
        if err != self.last_error:
            print(f"[{self.name}] {where} error: {err}", file=sys.stderr, flush=True)
        self.last_error = err

    def run(self):
        try:
            self.setup()
        except Exception:
            self._report("setup")
            self.failed = True
            return
        while not self._stop_evt.is_set():
            try:
                self.poll()
            except Exception:
                self._report("poll")
                self._stop_evt.wait(0.05)
        try:
            self.teardown()
        except Exception:
            self._report("teardown")

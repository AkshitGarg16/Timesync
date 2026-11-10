import math
import time
from dataclasses import dataclass


class Clock:

    def __init__(self):
        self.refresh()
        self.boot_time = self.mono_ns()
        self.boot_unix_time = self.to_unix_ns(self.boot_time)

    def refresh(self):
        m0 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        r = time.clock_gettime_ns(time.CLOCK_REALTIME)
        m1 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        self._offset_ns = r - (m0 + m1) // 2
        self.capture_uncertainty_ns = (m1 - m0) // 2

    @staticmethod
    def mono_ns() -> int:
        return time.monotonic_ns()

    def to_unix_ns(self, mono_ns: int) -> int:
        return mono_ns + self._offset_ns

    
@dataclass
class ClockOffset:

    alpha_initial: float = 0.05
    beta_initial: float = 0.05

    alpha_final: float = 0.003
    beta_final: float = 0.003
    
    convergence_window: int = 300
    
    max_rtt_ns: int = 10_000_000
    
    max_deviation_ns: int = 100_000_000
    max_consecutive_high_deviation: int = 5

    offset_ns: float = 0.0
    skew_ns: float = 0.0
    sequence: int = 0
    accepted: int = 0
    rejected: int = 0
    last_rtt_ns: int = 0
    _high_dev: int = 0

    @property
    def converged(self) -> bool:
        return self.sequence >= self.convergence_window

    def reset(self):
        self.offset_ns = self.skew_ns = 0.0
        self.sequence = self._high_dev = 0

    def _gains(self):
        if self.converged:
            return self.alpha_final, self.beta_final
        p_raw = self.sequence / self.convergence_window
        p = 1.0 - math.exp(0.5 * (1.0 - 1.0 / (1.0 - p_raw))) if p_raw < 1 else 1.0
        return (p * self.alpha_final + (1 - p) * self.alpha_initial,
                p * self.beta_final + (1 - p) * self.beta_initial)

    def observe_roundtrip(self, sent_ns: int, device_ns: int, recv_ns: int) -> bool:
        rtt = recv_ns - sent_ns
        self.last_rtt_ns = rtt
        if rtt <= 0 or rtt > self.max_rtt_ns:
            self.rejected += 1
            return False
        return self._update((sent_ns + recv_ns - 2 * device_ns) / 2.0)

    def observe_oneway(self, device_ns: int, arrival_ns: int) -> bool:
        return self._update(float(arrival_ns - device_ns))

    def _update(self, offset: float) -> bool:
        if self.converged and abs(self.offset_ns - offset) > self.max_deviation_ns:
            self._high_dev += 1
            self.rejected += 1
            if self._high_dev > self.max_consecutive_high_deviation:
                self.reset()
            return False

        alpha, beta = self._gains()
        prev = self.offset_ns
        if self.sequence == 0:
            self.offset_ns = offset
        else:
            self.offset_ns = alpha * offset + (1 - alpha) * (self.offset_ns + self.skew_ns)
            self.skew_ns = beta * (self.offset_ns - prev) + (1 - beta) * self.skew_ns
        self.sequence += 1
        self.accepted += 1
        self._high_dev = 0
        return True

    def to_host(self, device_ns: int, arrival_ns: int) -> int:
        if not self.converged:
            return arrival_ns
        return int(device_ns + self.offset_ns)


class Uint32Unwrapper:

    def __init__(self, scale_to_ns: int = 1_000_000):
        self._epochs = 0
        self._last = None
        self._scale = scale_to_ns

    def to_ns(self, value: int) -> int:
        if self._last is not None and value < self._last - (1 << 31):
            self._epochs += 1
        self._last = value
        return (value + self._epochs * (1 << 32)) * self._scale
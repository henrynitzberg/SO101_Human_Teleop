import statistics
import time
from collections import deque

class RateMonitor:
    def __init__(self, target_hz: float, window: int = 200, thresh: float = 0.5):
        self.target = target_hz
        self.window = window
        self.periods = deque(maxlen=window)
        self.overruns = 0
        self.last = None
        self.thresh = thresh

    def tick(self):
        now = time.perf_counter()
        if self.last is not None:
            dt = now - self.last
            self.periods.append(dt)
            if dt > (1.0 / self.target) * (1 + self.thresh):
                self.overruns += 1
        self.last = now

    def report(self) -> str:
        if len(self.periods) < 2:
            return "no data"
        p = sorted(self.periods)
        mean = statistics.mean(p)
        return (
            f"target: {self.target:.2f} Hz, "
            f"mean: {1/mean:.2f} Hz, "
            f"min: {1/p[-1]:.2f} Hz, "
            f"max: {1/p[0]:.2f} Hz"
        )
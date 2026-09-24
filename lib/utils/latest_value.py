import threading
import time

class LatestValue:
    def __init__(self):
        self._lock = threading.Lock()
        self._value = None
        self._stamp = 0.0
        self._seq = 0

    def put(self, value) -> None:
        with self._lock:
            self._value = value
            self._stamp = time.perf_counter()
            self._seq += 1

    def get(self):
        with self._lock:
            if self._value is None:
                return None, float('inf'), 0
            return self._value, time.perf_counter() - self._stamp, self._seq


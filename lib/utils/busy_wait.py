import time


def busy_wait(seconds: float):
    if seconds <= 0:
        return
    end = time.perf_counter() + seconds

    coarse = seconds - 0.001
    if coarse > 0:
        time.sleep(coarse)
    while time.perf_counter() < end:
        pass

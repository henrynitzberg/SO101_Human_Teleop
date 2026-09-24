import math

def approach(current: dict, target: dict, tau: float, dt: float, max_rate: float) -> dict:
    """Move `current` toward `target`, one key at a time."""
    step = max_rate * dt
    alpha = 1.0 - math.exp(-dt / tau)
    result = dict(current)
    for k, t in target.items():
        c = current[k]
        smoothed = c + alpha * (t - c)
        result[k] = c + max(-step, min(step, smoothed - c))
    return result

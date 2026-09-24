import time
from lib.utils.busy_wait import busy_wait


def interpolate(start: dict, goal: dict, alpha: float) -> dict:
    return {k: (1 - alpha) * start[k] + alpha * goal[k] for k in goal}


def smoothstep(t: float) -> float:
    # t^2 * (3 - 2t) = 3t^2 - 2t^3
    return t * t * (3 - 2 * t)


def move_to(robot, goal: dict, duration: float = 2.0, fps: int = 50):
    """Scripted move between the robot's current pose and a fixed `goal`
    over `duration` seconds."""
    obs = robot.get_observation()
    start = {k: obs[k] for k in goal}

    steps = max(1, int(duration * fps))
    for i in range(steps + 1):
        t0 = time.perf_counter()
        alpha = smoothstep(i / steps)
        robot.send_action(interpolate(start, goal, alpha))
        busy_wait(1 / fps - (time.perf_counter() - t0))

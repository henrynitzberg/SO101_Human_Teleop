import sys
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lib.movement.smooth_move import move_to
from lib.movement.robot_config import make_config

config = make_config()
with SO101Follower(config) as robot:
    obs = robot.get_observation()
    base = obs["shoulder_pan.pos"]
    for target in [base + 30, base, base - 30, base]:
        move_to(robot, {"shoulder_pan.pos": target}, duration=1)

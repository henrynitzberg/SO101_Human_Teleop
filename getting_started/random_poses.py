import random
import sys
import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lib.movement.smooth_move import move_to
from lib.movement.load_poses import load_poses
from lib.movement.robot_config import make_config

POSE_COUNT = 3

config = make_config()
poses = load_poses("poses")
if not poses:
    raise RuntimeError(
        "No poses available in 'poses/' - record at least one with generate_pose.py."
    )

# random.choices samples with replacement, so this still produces exactly
# POSE_COUNT moves even if fewer than POSE_COUNT poses exist (repeating some).
chosen = random.choices(list(poses.items()), k=POSE_COUNT)

with SO101Follower(config) as robot:
    print("Connected:", robot.is_connected)

    for pose_name, pose in chosen:
        print(f"Moving to pose '{pose_name}': {pose}")
        move_to(robot, pose, duration=1.0)
        time.sleep(0.5)

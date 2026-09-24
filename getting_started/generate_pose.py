import sys
import threading
import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lib.movement.load_poses import save_pose
from lib.movement.robot_config import make_config

pose_name = input("Enter pose name: ")

config = make_config()

with SO101Follower(config) as robot:
    print("Move the robot into the desired pose.")
    robot.bus.disable_torque()

    stop_event = threading.Event()

    def poll_observations():
        while not stop_event.is_set():
            obs = robot.get_observation()
            print(f"Current observation: {obs}")
            time.sleep(0.1)

    poll_thread = threading.Thread(target=poll_observations, daemon=True)
    poll_thread.start()

    input("Press Enter to record pose: ")

    stop_event.set()
    poll_thread.join()

    obs = robot.get_observation()
    pose = {k: obs[k] for k in obs if k.endswith(".pos")}
    save_pose("poses", pose_name, pose)

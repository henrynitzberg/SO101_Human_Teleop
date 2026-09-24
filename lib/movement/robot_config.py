import os
from pathlib import Path

from dotenv import load_dotenv
from lerobot.robots.so_follower import SO101FollowerConfig

load_dotenv(Path(__file__).resolve().parent.parent.parent / ".env")

try:
    PORT = os.environ["SO101_PORT"]
    ROBOT_ID = os.environ["SO101_ROBOT_ID"]
except KeyError as e:
    raise RuntimeError(
        f"Missing {e.args[0]} - copy .env.example to .env and fill in your robot's port/id."
    ) from e


def make_config(**overrides):
    """SO101FollowerConfig for this robot, with the project's standard
    teleop-safe defaults. Pass overrides (e.g. max_relative_target=...) to
    change any of them for a specific script."""
    return SO101FollowerConfig(
        port=PORT,
        id=ROBOT_ID,
        max_relative_target=20.0,
        disable_torque_on_disconnect=True,
        **overrides,
    )

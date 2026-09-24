import json
import os


def load_poses(pose_dir):
    pose_names = [f[:-5] for f in os.listdir(pose_dir) if f.endswith(".json")]
    poses = {}
    for pose_name in pose_names:
        with open(os.path.join(pose_dir, f"{pose_name}.json"), "r") as f:
            poses[pose_name] = json.load(f)
    return poses


def save_pose(pose_dir, name, data):
    os.makedirs(pose_dir, exist_ok=True)
    path = os.path.join(pose_dir, f"{name}.json")
    with open(path, "w") as f:
        json.dump(data, f, indent=4)
    print(f"Pose saved to {path}")

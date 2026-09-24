import argparse
import contextlib
import json
import math
import threading

from lerobot.robots.so_follower.so_follower import SO101Follower
from lerobot.robots.so_follower.robot_kinematic_processor import (
    EEReferenceAndDelta,
    InverseKinematicsEEToJoints,
    EEBoundsAndSafety,
)
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import (
    RobotProcessorPipeline,
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.utils.rotation import Rotation

from follow_helpers import (
    HandLandmark,
    angle_to_pos,
    GripAngleTracker,
    draw_hand_skeleton,
    get_wrist_point,
    get_landmark_point,
)
from lib.utils.busy_wait import busy_wait
from lib.vision.get_detector import get_hand_detector, detect_hand
from lib.angle_calc.hand_frame import (
    compute_hand_frame,
    compute_heading,
    signed_angle_about_axis,
)
from lib.utils.one_euro_filter import OneEuroFilter
from lib.utils.rate_monitor import RateMonitor
from lib.utils.latest_value import LatestValue
from lib.movement.approach import approach
from lib.movement.load_poses import load_poses
from lib.movement.smooth_move import move_to
from lib.vision.cli_args import add_calibration_arg, add_camera_args
from lib.vision.load_calibration import load_stereo_calibration
from lib.vision.status_ui import (
    combine_side_by_side,
    draw_status_banner,
    draw_status_panel,
)
from lib.vision.triangulate import triangulate_point
from lib.angle_calc.fuse import fuse_grip_angles
from lib.movement.robot_config import make_config
import time

import cv2
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument(
    "-r",
    "--robotless",
    action="store_true",
    help="Run the vision pipeline only, without connecting to the robot.",
)
add_camera_args(parser)
add_calibration_arg(parser)
parser.add_argument(
    "--basis-vectors",
    type=str,
    default="calibration/data/basis_vectors.json",
    help="forward/up unit vectors JSON from calibration/record_forward_base_vec.py/record_up_base_vec.py.",
)
args = parser.parse_args()

# Max age (seconds) before a camera's reading is considered stale.
MAX_READING_AGE = 0.5

with open(args.basis_vectors) as f:
    _basis = json.load(f)
FORWARD_UNIT = np.array(
    _basis["forward"]["unit"]
)  # camera A frame, from record_forward_base_vec.py
UP_UNIT = np.array(_basis["up"]["unit"])
ROLL_AXIS = np.array(
    _basis["roll"]["axis"]
)  # camera A frame, from record_roll_base_vec.py
PITCH_AXIS = np.array(
    _basis["pitch"]["axis"]
)  # camera A frame, from record_pitch_base_vec.py

# "Forward" projected into the horizontal plane (perpendicular to UP_UNIT)
# and renormalized - the fixed reference yaw is measured against.
FORWARD_HORIZONTAL = FORWARD_UNIT - np.dot(FORWARD_UNIT, UP_UNIT) * UP_UNIT
FORWARD_HORIZONTAL = FORWARD_HORIZONTAL / np.linalg.norm(FORWARD_HORIZONTAL)

EE_BOUNDS = {
    "min": [0.05, -0.25, 0.01],
    "max": [0.35, 0.25, 0.30],
}

# Direction convention only (not derived from kinematics) - flip any of
# these if the joint turns opposite to your hand.
ROLL_DIRECTION = -1.0
PITCH_DIRECTION = 1.0
YAW_DIRECTION = -1.0

# the robot's wrist_flex range is much wider than a human wrist's real flex, so
# we clamp the robots range of motion.ull
PITCH_MIN_DEG = -30.0
PITCH_MAX_DEG = 70.0

# wrist_flex's own hardware range (from urdf/so101.urdf: lower/upper =
# +/-1.65806 rad), as a final safety clamp on wrist_flex_target - the
# posture-compensation term below (offsetting for however far shoulder_lift/
# elbow_flex have moved) is in principle unbounded if the arm reaches to an
# extreme posture, so this is a hard backstop independent of PITCH_MIN/MAX_DEG.
WRIST_FLEX_LIMIT_DEG = 95.0

# Minimum horizontal-projection magnitude (of a unit vector) before treating
# the yaw angle as too unstable to update - guards the case where the hand
# faces nearly straight up/down (heading nearly parallel to UP_UNIT).
MIN_HORIZONTAL_MAGNITUDE = 0.2

# OneEuroFilter params for the 5 scalars that actually drive the robot
# (target_x, target_z, roll_amount, pitch_amount, yaw_delta) - see
# lib/utils/one_euro_filter.py.
# Per the paper's tuning procedure: start with beta=0 and lower min_cutoff
# until idle jitter is acceptable, then raise beta until lag during fast
# movement is acceptable.
POSITION_MIN_CUTOFF = 1.0  # target_x/target_z are in meters
POSITION_BETA = 0.0
ROTATION_MIN_CUTOFF = 1.0  # roll/pitch/yaw amounts are in radians
ROTATION_BETA = 0.0
FILTER_D_CUTOFF = 1.0


def _make_filter(now, min_cutoff, beta):
    return OneEuroFilter(
        now, 0.0, min_cutoff=min_cutoff, beta=beta, d_cutoff=FILTER_D_CUTOFF
    )


def camera_producer(
    cam_id: str,
    cap: cv2.VideoCapture,
    hand_detector,
    tracker: GripAngleTracker,
    angle_box: LatestValue,
    point_box: LatestValue,
    orientation_box: LatestValue,
    frame_box: LatestValue,
    stop: threading.Event,
):
    try:
        while not stop.is_set():
            success, frame = cap.read()
            if not success:
                print(f"[{cam_id}] Video playback finished or failed to read frame.")
                stop.set()
                break

            hand_detection_result = detect_hand(frame, hand_detector)
            h, w = frame.shape[:2]

            gripper_angle = tracker.update(hand_detection_result, w, h)
            if gripper_angle is not None:
                angle_box.put(gripper_angle)

            wrist_point = get_wrist_point(hand_detection_result, w, h)
            if wrist_point is not None:
                point_box.put(wrist_point)

            index_point = get_landmark_point(
                hand_detection_result, HandLandmark.INDEX_FINGER_MCP, w, h
            )
            pinky_point = get_landmark_point(
                hand_detection_result, HandLandmark.PINKY_MCP, w, h
            )
            if (
                wrist_point is not None
                and index_point is not None
                and pinky_point is not None
            ):
                orientation_box.put((wrist_point, index_point, pinky_point))

            annotated = frame.copy()
            draw_hand_skeleton(annotated, hand_detection_result, w, h, (255, 255, 255))
            frame_box.put(annotated)
    finally:
        cap.release()


hand_detector_a = get_hand_detector()
hand_detector_b = get_hand_detector()
tracker_a = GripAngleTracker()
tracker_b = GripAngleTracker()
calib = load_stereo_calibration(args.calibration)

# Rate configuration
HZ = 50
PERIOD = 1.0 / HZ
rate = RateMonitor(HZ)

# Camera configuration
cap_a = cv2.VideoCapture(args.cam_a)
cap_b = cv2.VideoCapture(args.cam_b)
if not cap_a.isOpened() or not cap_b.isOpened():
    raise RuntimeError("Could not open both cameras. Check --cam-a/--cam-b indices.")
angle_box_a = LatestValue()
angle_box_b = LatestValue()
point_box_a = LatestValue()
point_box_b = LatestValue()
orientation_box_a = LatestValue()
orientation_box_b = LatestValue()
frame_box_a = LatestValue()
frame_box_b = LatestValue()
stop = threading.Event()

producer_a = threading.Thread(
    target=camera_producer,
    args=(
        "A",
        cap_a,
        hand_detector_a,
        tracker_a,
        angle_box_a,
        point_box_a,
        orientation_box_a,
        frame_box_a,
        stop,
    ),
    daemon=True,
)
producer_b = threading.Thread(
    target=camera_producer,
    args=(
        "B",
        cap_b,
        hand_detector_b,
        tracker_b,
        angle_box_b,
        point_box_b,
        orientation_box_b,
        frame_box_b,
        stop,
    ),
    daemon=True,
)
producer_a.start()
producer_b.start()

# robot boilerplate
poses = load_poses("poses")
config = make_config()

robot_cm = contextlib.nullcontext(None) if args.robotless else SO101Follower(config)

with robot_cm as robot:
    if robot:
        motor_names = list(robot.bus.motors)
        robot.bus.sync_write("Maximum_Acceleration", {m: 0 for m in motor_names})

        move_to(robot, poses["look_forward"], duration=4.0)
        obs = robot.get_observation()
        smoothed = {"gripper.pos": obs["gripper.pos"]}
        # IK places the wrist_flex pivot at the tracked hand position (not the
        # gripper directly) so that pitch and roll can be manipulated
        # seperately.
        kinematics = RobotKinematics(
            urdf_path="./urdf/so101.urdf",
            target_frame_name="wrist_link",
            joint_names=motor_names,
        )
        for masked_joint in ("shoulder_pan", "wrist_roll"):
            kinematics.solver.mask_dof(masked_joint)
        teleop_processor = RobotProcessorPipeline[tuple[dict, dict], dict](
            steps=[
                EEReferenceAndDelta(
                    kinematics=kinematics,
                    end_effector_step_sizes={"x": 1.0, "y": 1.0, "z": 1.0},
                    motor_names=motor_names,
                    use_latched_reference=True,
                ),
                EEBoundsAndSafety(
                    end_effector_bounds=EE_BOUNDS,
                    max_ee_step_m=0.02,
                    raise_on_jump=False,
                ),
                InverseKinematicsEEToJoints(
                    kinematics=kinematics,
                    motor_names=motor_names,
                    initial_guess_current_joints=True,
                    orientation_weight=0.0,  # position-only IK on wrist_link
                ),
            ],
            to_transition=robot_action_observation_to_transition,
            to_output=transition_to_robot_action,
        )

    next_deadline = time.perf_counter()
    last_t = next_deadline
    last_print = 0.0
    last_xyz = None
    last_hand_rotation = None
    last_yaw = None
    last_fused = None
    position_tracking_enabled = False
    wrist_origin = None
    hand_rotation_origin = None
    yaw_origin = None
    shoulder_pan_reference = None
    wrist_roll_reference = None
    shoulder_lift_reference = None
    elbow_flex_reference = None
    wrist_flex_reference = None
    filter_target_x = None
    filter_target_z = None
    filter_roll = None
    filter_pitch = None
    filter_yaw = None

    try:
        while not stop.is_set():
            next_deadline += PERIOD
            rate.tick()

            now = time.perf_counter()
            dt = now - last_t
            last_t = now

            angle_a, angle_age_a, _ = angle_box_a.get()
            angle_b, angle_age_b, _ = angle_box_b.get()
            fused = fuse_grip_angles(
                angle_a,
                angle_age_a,
                angle_b,
                angle_age_b,
                last_fused,
                max_age=MAX_READING_AGE,
            )
            if fused is not None:
                last_fused = fused

            point_a, point_age_a, _ = point_box_a.get()
            point_b, point_age_b, _ = point_box_b.get()
            if (
                point_a is not None
                and point_age_a < MAX_READING_AGE
                and point_b is not None
                and point_age_b < MAX_READING_AGE
            ):
                last_xyz = triangulate_point(calib, point_a, point_b)

            orient_a, orient_age_a, _ = orientation_box_a.get()
            orient_b, orient_age_b, _ = orientation_box_b.get()
            if (
                orient_a is not None
                and orient_age_a < MAX_READING_AGE
                and orient_b is not None
                and orient_age_b < MAX_READING_AGE
            ):
                wrist_3d = triangulate_point(calib, orient_a[0], orient_b[0])
                index_3d = triangulate_point(calib, orient_a[1], orient_b[1])
                pinky_3d = triangulate_point(calib, orient_a[2], orient_b[2])
                last_hand_rotation = compute_hand_frame(wrist_3d, index_3d, pinky_3d)

                heading = compute_heading(wrist_3d, index_3d, pinky_3d)
                heading_horizontal = heading - np.dot(heading, UP_UNIT) * UP_UNIT
                heading_horizontal_norm = np.linalg.norm(heading_horizontal)
                if heading_horizontal_norm >= MIN_HORIZONTAL_MAGNITUDE:
                    heading_horizontal = heading_horizontal / heading_horizontal_norm
                    last_yaw = signed_angle_about_axis(
                        FORWARD_HORIZONTAL, heading_horizontal, UP_UNIT
                    )

            if robot:
                obs = robot.get_observation()

                # Roll/pitch (and the gripper, gated separately below) track
                # the hand continuously, independent of position_tracking_enabled -
                # only forward/up position and yaw are clutch-gated.
                if hand_rotation_origin is None and last_hand_rotation is not None:
                    hand_rotation_origin = last_hand_rotation.copy()
                    wrist_roll_reference = obs["wrist_roll.pos"]
                    shoulder_lift_reference = obs["shoulder_lift.pos"]
                    elbow_flex_reference = obs["elbow_flex.pos"]
                    wrist_flex_reference = obs["wrist_flex.pos"]
                    filter_roll = _make_filter(now, ROTATION_MIN_CUTOFF, ROTATION_BETA)
                    filter_pitch = _make_filter(now, ROTATION_MIN_CUTOFF, ROTATION_BETA)

                if fused is not None:
                    smoothed = approach(
                        smoothed,
                        {"gripper.pos": angle_to_pos(fused, 15, 90, 0, 100)},
                        tau=0.1,
                        dt=dt,
                        max_rate=100.0,
                    )

                if (
                    position_tracking_enabled
                    and last_xyz is not None
                    and wrist_origin is not None
                ):
                    delta_camera = last_xyz - wrist_origin
                    target_x = float(np.dot(delta_camera, FORWARD_UNIT))
                    target_z = float(np.dot(delta_camera, UP_UNIT))
                    target_x = filter_target_x(now, target_x)
                    target_z = filter_target_z(now, target_z)
                    wrist_enabled = True
                else:
                    target_x = 0.0
                    target_z = 0.0
                    wrist_enabled = False

                if (
                    position_tracking_enabled
                    and last_yaw is not None
                    and yaw_origin is not None
                    and shoulder_pan_reference is not None
                ):
                    yaw_delta = last_yaw - yaw_origin
                    yaw_delta = (yaw_delta + math.pi) % (2 * math.pi) - math.pi
                    yaw_delta = filter_yaw(now, yaw_delta)
                    shoulder_pan_target = (
                        shoulder_pan_reference + math.degrees(yaw_delta) * YAW_DIRECTION
                    )
                else:
                    shoulder_pan_target = None

                if last_hand_rotation is not None and hand_rotation_origin is not None:
                    r_delta_camera = last_hand_rotation @ hand_rotation_origin.T
                    rotvec_camera = Rotation.from_matrix(r_delta_camera).as_rotvec()

                    if wrist_roll_reference is not None:
                        roll_amount = float(np.dot(rotvec_camera, ROLL_AXIS))
                        roll_amount = filter_roll(now, roll_amount)
                        wrist_roll_target = (
                            wrist_roll_reference
                            + math.degrees(roll_amount) * ROLL_DIRECTION
                        )
                    else:
                        wrist_roll_target = None

                    pitch_amount = float(np.dot(rotvec_camera, PITCH_AXIS))
                    pitch_amount = filter_pitch(now, pitch_amount)
                    pitch_offset_deg = math.degrees(pitch_amount) * PITCH_DIRECTION
                    pitch_offset_deg = max(
                        PITCH_MIN_DEG, min(PITCH_MAX_DEG, pitch_offset_deg)
                    )
                else:
                    wrist_roll_target = None
                    pitch_offset_deg = 0.0

                wrist_action = {
                    "enabled": wrist_enabled,
                    "target_x": target_x,
                    "target_y": 0.0,
                    "target_z": target_z,
                    "target_wx": 0.0,
                    "target_wy": 0.0,
                    "target_wz": 0.0,
                    "gripper_vel": 0.0,
                    "ee.gripper_pos": smoothed["gripper.pos"],
                }
                command = teleop_processor((wrist_action, obs))
                if shoulder_pan_target is not None:
                    command["shoulder_pan.pos"] = shoulder_pan_target
                if wrist_roll_target is not None:
                    command["wrist_roll.pos"] = wrist_roll_target
                # Solve for the wrist_flex that keeps the gripper's absolute
                # pitch matching the hand's, compensating for however far
                # shoulder_lift/elbow_flex have moved since the reference
                # was latched - see the comment above kinematics.solver.
                if wrist_flex_reference is not None:
                    wrist_flex_target = (
                        wrist_flex_reference
                        + pitch_offset_deg
                        - (command["shoulder_lift.pos"] - shoulder_lift_reference)
                        - (command["elbow_flex.pos"] - elbow_flex_reference)
                    )
                    wrist_flex_target = max(
                        -WRIST_FLEX_LIMIT_DEG,
                        min(WRIST_FLEX_LIMIT_DEG, wrist_flex_target),
                    )
                    command["wrist_flex.pos"] = wrist_flex_target

                robot.send_action(command)

            display_frame_a, _, _ = frame_box_a.get()
            display_frame_b, _, _ = frame_box_b.get()
            if display_frame_a is not None and display_frame_b is not None:
                combined = combine_side_by_side(display_frame_a, display_frame_b)

                angle_text = f"{int(fused)} deg" if fused is not None else "--"
                draw_status_panel(combined, [(f"angle: {angle_text}", 1.0)])

                status_text = (
                    "Position/yaw: engaged"
                    if position_tracking_enabled
                    else "Position/yaw: disengaged (SPACE to engage) - roll/pitch/gripper always track"
                )
                draw_status_banner(
                    combined,
                    status_text,
                    center_x=combined.shape[1] // 2,
                    max_width=combined.shape[1] * 0.9,
                )

                cv2.imshow("Pose Estimation", combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                stop.set()
            elif key == ord(" "):
                if position_tracking_enabled:
                    position_tracking_enabled = False
                    print("Position/yaw tracking disengaged.")
                elif (
                    last_xyz is not None
                    and last_hand_rotation is not None
                    and last_yaw is not None
                ):
                    position_tracking_enabled = True
                    wrist_origin = last_xyz.copy()
                    hand_rotation_origin = last_hand_rotation.copy()
                    yaw_origin = last_yaw
                    shoulder_pan_reference = obs["shoulder_pan.pos"] if robot else None
                    # Also refresh roll/pitch's origin, even though they're
                    # already tracking continuously (see the auto-latch
                    # above) - a nice bonus recalibration each time you
                    # engage, in case your hand drifted from its resting
                    # orientation since the last refresh.
                    wrist_roll_reference = obs["wrist_roll.pos"] if robot else None
                    shoulder_lift_reference = (
                        obs["shoulder_lift.pos"] if robot else None
                    )
                    elbow_flex_reference = obs["elbow_flex.pos"] if robot else None
                    wrist_flex_reference = obs["wrist_flex.pos"] if robot else None
                    # Fresh filters each engage - all 5 signals are deltas
                    # from the just-latched origins, so they start at 0.0.
                    filter_target_x = _make_filter(
                        now, POSITION_MIN_CUTOFF, POSITION_BETA
                    )
                    filter_target_z = _make_filter(
                        now, POSITION_MIN_CUTOFF, POSITION_BETA
                    )
                    filter_roll = _make_filter(now, ROTATION_MIN_CUTOFF, ROTATION_BETA)
                    filter_pitch = _make_filter(now, ROTATION_MIN_CUTOFF, ROTATION_BETA)
                    filter_yaw = _make_filter(now, ROTATION_MIN_CUTOFF, ROTATION_BETA)
                    print("Position/yaw tracking engaged.")
                else:
                    print("Can't engage position/yaw tracking yet - no hand tracked.")

            now = time.perf_counter()
            if now - last_print > 1.0:
                if last_xyz is not None:
                    print(
                        f"wrist (m): x={last_xyz[0]:+.3f} y={last_xyz[1]:+.3f} z={last_xyz[2]:+.3f}"
                    )
                last_print = now
            if now > next_deadline:
                next_deadline = now
            else:
                busy_wait(next_deadline - now)
    finally:
        stop.set()
        if robot:
            move_to(robot, poses["home"], duration=5.0)

producer_a.join(timeout=1.0)
producer_b.join(timeout=1.0)
cv2.destroyAllWindows()

import time

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


def get_hand_detector():
    hand_base_options = python.BaseOptions(
        model_asset_path="models/hand_landmarker.task",
        delegate=python.BaseOptions.Delegate.CPU,
    )
    hand_options = vision.HandLandmarkerOptions(
        base_options=hand_base_options,
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
    )
    hand_detector = vision.HandLandmarker.create_from_options(hand_options)
    return hand_detector


def detect_hand(frame, hand_detector):
    """Run one BGR video frame through a MediaPipe HandLandmarker in VIDEO
    mode."""
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    now_ms = int(time.perf_counter() * 1000)
    return hand_detector.detect_for_video(image, now_ms)

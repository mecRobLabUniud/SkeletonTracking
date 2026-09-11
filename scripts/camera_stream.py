#!/usr/bin/env python3

"""
░█▀▀░█▀█░█▄█░█▀▀░█▀▄░█▀█░░░█▀▀░▀█▀░█▀▄░█▀▀░█▀█░█▄█
░█░░░█▀█░█░█░█▀▀░█▀▄░█▀█░░░▀▀█░░█░░█▀▄░█▀▀░█▀█░█░█
░▀▀▀░▀░▀░▀░▀░▀▀▀░▀░▀░▀░▀░░░▀▀▀░░▀░░▀░▀░▀▀▀░▀░▀░▀░▀

Script for real-time skeleton tracking using RealSense D435 and YOLOv8-Pose, 
with data streaming via ZeroMQ for use in a robot control loop (e.g. admittance control). 
It includes robust depth reading, temporal smoothing of keypoints, and simplified 
capsule representation of limbs for collision avoidance. 
The script supports multiple cameras and can save video output for debugging.
"""

import signal
import os
import numpy as np
import time
import logging
import pyrealsense2 as rs
from utils.skeleton_tracker import SkeletonTracker
from utils.data_transmitter import DataTransmitter
from utils.decorators import chronometer, set_rate

logging.getLogger('ultralytics').setLevel(logging.ERROR)
logging.getLogger('tensorrt').setLevel(logging.ERROR)

# ─────────────────────────────────────────────────────────────────────────────
# Parameters
# ─────────────────────────────────────────────────────────────────────────────
running = True
save_data = False
display_stream = False
script_dir = os.path.dirname(os.path.abspath(__file__))
yolo_model = "yolo26n-pose"
yolo_hand_model = "yolo26n-hands"


# ─────────────────────────────────────────────────────────────────────────────
# Load pose matrix
# ─────────────────────────────────────────────────────────────────────────────
def load_rotation_matrix(path_txt):
    T = np.loadtxt(path_txt, dtype=np.float64)
    assert T.shape == (4, 4)
    return T


# ─────────────────────────────────────────────────────────────────────────────
# Coordinates transformation
# ─────────────────────────────────────────────────────────────────────────────
def transform_points(T, pts_skeleton):
    pts_h = np.concatenate([pts_skeleton, np.ones((pts_skeleton.shape[0], 1))], axis=1)
    return (T @ pts_h.T).T[:, :3]


# ─────────────────────────────────────────────────────────────────────────────
# Skeleton tracking
# ─────────────────────────────────────────────────────────────────────────────
@set_rate(60)
def tracking(dtss, trackers, rotation_matrices):    
    for n, (dts, tracker, rotation_matrix) in enumerate(zip(dtss, trackers, rotation_matrices)):
        frame = tracker.read_frame()
        skeleton, confidence = tracker.read_coords()   

        # Write frame into shared memory
        if not frame is None:
            dts.send_frames(frame)
        
        # Write skeleton data into socket
        if not skeleton is None and not confidence is None:
            skeleton = transform_points(rotation_matrix, skeleton.astype(np.float64)).astype(np.float32)
            confidence = confidence.astype(np.float32)

            dts.send_data(skeleton, confidence)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point 
# ─────────────────────────────────────────────────────────────────────────────
def main():
    ctx = rs.context()
    devices = ctx.devices
    dtss = []
    trackers = []
    rotation_matrices = []
    for n, device in enumerate(devices):
        # Sender initialization
        dts = DataTransmitter("sender", n, "SINGLE_CAMERA")
        dtss.append(dts)

        # Create trackers
        tracker = SkeletonTracker(device.get_info(rs.camera_info.serial_number))
        tracker.start("pose")
        trackers.append(tracker)

        serial = tracker.get_serial_number()
        rotation_matrix = load_rotation_matrix(os.path.join(script_dir, f"data/calibration/pose_{serial}.txt"))
        rotation_matrices.append(rotation_matrix)

        print(f"Device {n} initialized: {device.get_info(rs.camera_info.name)} (SN: {device.get_info(rs.camera_info.serial_number)})")
    print("Streaming started")

    # Clear shutdown logic
    def signal_handler(sig, frame):
        global running
        running = False
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Main loop
    while running:
        tracking(dtss, trackers, rotation_matrices)

    for (dts, tracker) in zip(dtss, trackers):
        dts.shutdown()
        tracker.shutdown()
    

if __name__ == "__main__":
    main()

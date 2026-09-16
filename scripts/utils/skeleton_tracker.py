#!/usr/bin/env python3
 
"""
░█▀▀░█░█░█▀▀░█░░░█▀▀░▀█▀░█▀█░█▀█░░░▀█▀░█▀▄░█▀█░█▀▀░█░█░█▀▀░█▀▄
░▀▀█░█▀▄░█▀▀░█░░░█▀▀░░█░░█░█░█░█░░░░█░░█▀▄░█▀█░█░░░█▀▄░█▀▀░█▀▄
░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░░░░▀░░▀░▀░▀░▀░▀▀▀░▀░▀░▀▀▀░▀░▀

Script for real-time skeleton tracking using RealSense D435 and YOLOv8-Pose, 
with data streaming via ZeroMQ for use in a robot control loop (e.g. admittance control). 
It includes robust depth reading, temporal smoothing of keypoints, and simplified 
capsule representation of limbs for collision avoidance.
"""

import cv2
import math
import time
import numpy as np
import pyrealsense2 as rs
import threading
import logging
from utils.filters import Keypoints3DSmoother
from mediapipe_utils.pose_inference_mediapipe import MediapipeTracker

logging.getLogger('ultralytics').setLevel(logging.ERROR)
logging.getLogger('tensorrt').setLevel(logging.ERROR)

# ─────────────────────────────────────────────────────────────────────────────
# Parameters
# ─────────────────────────────────────────────────────────────────────────────
conf_thr = 0.5
running = True


# ─────────────────────────────────────────────────────────────────────────────
# Skeleton tracker using YOLOv8-Pose for keypoint detection
# ─────────────────────────────────────────────────────────────────────────────
class SkeletonTracker:
    def __init__(self, device, w_camera: int=848, h_camera: int=480, camera_rate: int=60, depth: bool=True):
        self.device = device
        self.color = None
        self.depth = None
        self.frame = None
        self.camera_thread = None
        self.model_thread = None
        self.started = []
        self.xyz = None
        self.conf_thr = conf_thr
        self.conf = None
        self.smoother = Keypoints3DSmoother(num_kpts=17, min_cutoff=0.01, beta=10.0)

        self.mutex = threading.Lock()
        self.align = rs.align(rs.stream.color) # Allinea depth a color
        self.pipe = self.setup_camera_streaming(self.device, w_camera, h_camera, camera_rate, depth)
        self.mp = MediapipeTracker()
        self.camera_thread = threading.Thread(target=self.camera_streaming, args=())  
        

    # ─────────────────────────────────────────────────────────────────────────────
    # Default destructor
    # ─────────────────────────────────────────────────────────────────────────────
    def __del__(self):
        try:
            self.shutdown()
        except:
            pass


    # ─────────────────────────────────────────────────────────────────────────────
    # RealSense pipeline initialization 
    # ─────────────────────────────────────────────────────────────────────────────
    def setup_camera_streaming(self, serial, w_camera, h_camera, camera_rate, depth):
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        if depth:
            cfg.enable_stream(rs.stream.depth, w_camera, h_camera, rs.format.z16, camera_rate)
        cfg.enable_stream(rs.stream.color, w_camera, h_camera, rs.format.bgr8, camera_rate)
        pipe.start(cfg)
        return pipe


    # ─────────────────────────────────────────────────────────────────────────────
    # Start threads
    # ─────────────────────────────────────────────────────────────────────────────
    def start(self, model):
        # self.mutex = threading.Lock()
        if not "camera" in self.started:
            self.started.append("camera")
            self.camera_thread.start()

        if model in self.started:
            return
        self.started.append(model)
        self.model_thread = threading.Thread(target=self.skeleton_tracking, args=(model,))
        self.model_thread.start()
        return self
    

    # ─────────────────────────────────────────────────────────────────────────────
    # RGB and depth streaming from RealSense D435
    # ─────────────────────────────────────────────────────────────────────────────
    def camera_streaming(self):
        last_frame_number = -1
        while running and self.started:
            fs = self.pipe.wait_for_frames()
            fs = self.align.process(fs)
            depth = fs.get_depth_frame()
            color = fs.get_color_frame()

            if not depth or not color:
                continue

            frame_number = color.get_frame_number()
            if frame_number == last_frame_number:
                continue
            last_frame_number = frame_number

            self.W, self.H = depth.get_width(), depth.get_height()
            self.intr = depth.profile.as_video_stream_profile().intrinsics
            with self.mutex:
                self.color = color
                self.depth = depth


    # ─────────────────────────────────────────────────────────────────────────────
    # skeleton tracking prediction
    # ─────────────────────────────────────────────────────────────────────────────
    def skeleton_tracking(self, model):
        while running and (model in self.started):
            if not self.depth or not self.color:
                continue
            else:
                with self.mutex:
                    color = self.color
                    depth = self.depth              

            color_img = np.asanyarray(color.get_data())
            rgb_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
            results = self.mp.inference_pose_landmarker(rgb_img)
            annotated = self.mp.draw_landmarks_on_image(rgb_img, results)
            
            if not results.pose_landmarks == []:
                person = results.pose_landmarks[0]
                xyz = np.full((len(person), 3), np.nan, dtype=np.float32)
                conf = np.full((len(person)), 0, dtype=np.float32)
                for k, landmark in enumerate(person):
                    u = landmark.x*self.W
                    v = landmark.y*self.H
                    margin = 15
                    if u < margin or u > (self.W - margin) or v < margin or v > (self.H - margin):
                        continue

                    # Depth reading
                    z = self.robust_depth_median(depth, u, v)

                    if not math.isfinite(z):
                        continue
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intr, [u, v], z)

                    xyz[k] = np.array([X, Y, Z], dtype=np.float32)
                    conf[k] = landmark.visibility

                with self.mutex:
                    self.xyz = self.smoother.update(xyz, conf, conf_thr)
                    self.conf = conf

            with self.mutex:
                self.frame = annotated.copy()


    # ─────────────────────────────────────────────────────────────────────────────
    # Robust depth reading around a pixel 
    # ─────────────────────────────────────────────────────────────────────────────
    def robust_depth_median(self, depth_frame, u, v, R=6, max_dist=5.0):
        w, h = depth_frame.get_width(), depth_frame.get_height()
        uu, vv = int(round(u)), int(round(v)) # pixel centrali, round() arrotonda al più vicino intero
        zs = []
        # Aumentato R da 4 a 6 per avere più campioni su cui fare la mediana
        for dy in range(-R, R + 1):
            y = vv + dy
            if y < 0 or y >= h:
                continue
            for dx in range(-R, R + 1):
                x = uu + dx
                if x < 0 or x >= w:
                    continue
                z = depth_frame.get_distance(x, y)  # metri (classe.metodo() di pyrealsense2)
                # Filtra valori zero (invalidi) e valori troppo lontani (rumorosi)
                if z > 0.05 and z <= max_dist and math.isfinite(z):
                    zs.append(z)
        if not zs:
            return float("nan")
        zs.sort()
        return zs[len(zs) // 2]


    # ─────────────────────────────────────────────────────────────────────────────
    # Get methods
    # ─────────────────────────────────────────────────────────────────────────────
    def get_aligned_frames(self):
        depth = None
        color = None
        while depth is None and color is None:
            fs = self.pipe.wait_for_frames()
            fs = self.align.process(fs)
            depth = fs.get_depth_frame()
            color = fs.get_color_frame()

        depth = np.asanyarray(depth.get_data()) 
        color = np.asanyarray(color.get_data()) 
        return depth, color
    
    def get_depth_frame(self):
        depth = None
        while depth is None:
            fs = self.pipe.wait_for_frames()
            depth = fs.get_depth_frame()

        depth = np.asanyarray(depth.get_data()) 
        return depth
    
    def get_color_frame(self):
        color = None
        while color is None:
            fs = self.pipe.wait_for_frames()
            color = fs.get_color_frame()

        color = np.asanyarray(color.get_data()) 
        return color
    
    def get_serial_number(self):
        return self.device

    def get_intrinsics(self):
        color_stream = self.pipe.get_active_profile().get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        mtx = np.array([
            [intrinsics.fx, 0,             intrinsics.ppx],
            [0,              intrinsics.fy, intrinsics.ppy],
            [0,              0,             1]
        ], dtype=np.float64)
        dist = np.array(intrinsics.coeffs, dtype=np.float64) 
        return mtx, dist


    # ─────────────────────────────────────────────────────────────────────────────
    # Read methods
    # ─────────────────────────────────────────────────────────────────────────────
    def read_frame(self):
        with self.mutex:
            frame = self.frame.copy() if self.frame is not None else None
        return frame
    
    def read_coords(self):
        with self.mutex:
            xyz = self.xyz.copy() if self.xyz is not None else None
            conf = self.conf.copy() if self.conf is not None else None            
        return xyz, conf
    def shutdown(self):
        self.started = []
        self.camera_thread.join()
        if self.model_thread is not None:
            self.model_thread.join()
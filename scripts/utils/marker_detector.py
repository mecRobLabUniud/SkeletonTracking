#!/usr/bin/env python3

"""
░█▄█░█▀█░█▀▄░█░█░█▀▀░█▀▄░░░█▀▄░█▀▀░▀█▀░█▀▀░█▀▀░▀█▀░█▀█░█▀▄
░█░█░█▀█░█▀▄░█▀▄░█▀▀░█▀▄░░░█░█░█▀▀░░█░░█▀▀░█░░░░█░░█░█░█▀▄
░▀░▀░▀░▀░▀░▀░▀░▀░▀▀▀░▀░▀░░░▀▀░░▀▀▀░░▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀
"""

import numpy as np
import cv2
import cv2.aruco as aruco

# ─────────────────────────────────────────────────────────────────────────────
# Parameters
# ─────────────────────────────────────────────────────────────────────────────
single_dim = 0.144


# ─────────────────────────────────────────────────────────────────────────────
# Marker detector
# ─────────────────────────────────────────────────────────────────────────────
class MarkerDetector:
    def __init__(self, tracker):
        self.single_dim = single_dim
        self.tracker = tracker
        self.matrix_coefficients, self.distortion_coefficients = tracker.get_intrinsics()


    # ── Static calibration ──────────────────────────────────────────────────────────────
    def simple_calibration(self, marker_ID): 
        for _ in range(3):
            frame = self.tracker.get_color_frame()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
            dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters()  # new style
            detector = aruco.ArucoDetector(dictionary, parameters)

            # lists of ids and the corners belonging to each id
            corners, ids, _ = detector.detectMarkers(gray)
            rotation_matrix = None

            if np.all(ids is not None):
                axis = np.float32([[-0.01, -0.01, 0], [-0.01, 0.01, 0], [0.01, -0.01, 0], [0.01, 0.01, 0]]).reshape(-1, 3)

                # Estimate pose of each marker
                if ids[0] == marker_ID:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], self.single_dim, self.matrix_coefficients, self.distortion_coefficients)
                    
                    # Build 4x4 pose matrix [R | t; 0 0 0 1]
                    R_mat, _ = cv2.Rodrigues(rvec)
                    rotation_matrix = np.eye(4, dtype=np.float32)
                    rotation_matrix[:3, :3] = R_mat  # Rotation part
                    rotation_matrix[:3, 3] = tvec.flatten()  # Translation part
                    rotation_matrix = np.linalg.inv(rotation_matrix)

                    aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                    imgpts, _ = cv2.projectPoints(axis, rvec, tvec, self.matrix_coefficients,
                                                    self.distortion_coefficients)

                    cv2.drawFrameAxes(frame, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, length=0.1)
                    relativePoint = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
                    cv2.circle(frame, relativePoint, 2, (255, 255, 0))
            
            # Display the resulting frame
            cv2.imshow('frame', frame)
            key = cv2.waitKey() & 0xFF
            if key == ord('y'):
                return rotation_matrix
            elif key == ord('n'):
                continue
        return None

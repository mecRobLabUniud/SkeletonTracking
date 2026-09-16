#!/usr/bin/env python3

"""
░█▄█░█▀█░█▀▄░█░█░█▀▀░█▀▄░░░█▀▄░█▀▀░▀█▀░█▀▀░█▀▀░▀█▀░█▀█░█▀▄
░█░█░█▀█░█▀▄░█▀▄░█▀▀░█▀▄░░░█░█░█▀▀░░█░░█▀▀░█░░░░█░░█░█░█▀▄
░▀░▀░▀░▀░▀░▀░▀░▀░▀▀▀░▀░▀░░░▀▀░░▀▀▀░░▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import time

# ─────────────────────────────────────────────────────────────────────────────
# Parameters
# ─────────────────────────────────────────────────────────────────────────────
marker_IDs = [17, 21, 34, 42, 50]
dim = 0.05  # 0.147
single_dim = 0.144
pitch = 0.029
transformations = [np.eye(4) for _ in marker_IDs]   # transformation matrices from any marker frame to marker 34 frame (taken as the reference one)
transformations[0] = np.array([[0, -1, 0, 0],
                                [0, 0, -1, -pitch],
                                [1, 0, 0, -pitch],
                                [0, 0, 0, 1]], dtype=np.float32)
transformations[1] = np.array([[0, 0, -1, -pitch],
                                [0, 1, 0, 0],
                                [1, 0, 0, -pitch],
                                [0, 0, 0, 1]], dtype=np.float32)
transformations[3] = np.array([[0, 0, 1, pitch],
                                [0, -1, 0, 0],
                                [1, 0, 0, -pitch],
                                [0, 0, 0, 1]], dtype=np.float32)
transformations[4] = np.array([[0, 1, 0, 0],
                                [0, 0, 1, pitch],
                                [1, 0, 0, -pitch],
                                [0, 0, 0, 1]], dtype=np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Marker detector
# ─────────────────────────────────────────────────────────────────────────────
class MarkerDetector:
    def __init__(self, tracker):
        self.dim = dim
        half = self.dim/2
        self.single_dim = single_dim
        self.obj_points = np.array([
            [-half,  half, 0],  # top-left
            [ half,  half, 0],  # top-right
            [ half, -half, 0],  # bottom-right
            [-half, -half, 0],  # bottom-left
        ], dtype=np.float32)
        self.tracker = tracker
        self.matrix_coefficients, self.distortion_coefficients = tracker.get_intrinsics()


    # ── Helpers ──────────────────────────────────────────────────────────────────
    def smooth_pose(self, rvec, tvec, prev_rvec, prev_tvec, alpha=0.7):
        if prev_rvec is None:
            return rvec, tvec
        rvec_smooth = alpha * rvec + (1 - alpha) * prev_rvec
        tvec_smooth = alpha * tvec + (1 - alpha) * prev_tvec
        return rvec_smooth, tvec_smooth


    # ── Dynamic calibration ──────────────────────────────────────────────────────────────
    def dynamic_calibration(self, marker_ID): 
        while True:   
            # operations on the frame come here
            frame = self.tracker.get_color_frame()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
            dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters()  # new style
            detector = aruco.ArucoDetector(dictionary, parameters)

            # lists of ids and the corners belonging to each id
            corners, ids, _ = detector.detectMarkers(gray)

            rotation_matrix = None
            if np.all(ids is not None):
                zipped = zip(ids, corners)
                ids, corners = zip(*(sorted(zipped)))
                axis = np.float32([[-0.01, -0.01, 0], [-0.01, 0.01, 0], [0.01, -0.01, 0], [0.01, 0.01, 0]]).reshape(-1, 3)

                prev_rvec, prev_tvec = None, None
                # Estimate pose of each marker
                for i in range(len(ids)):
                    if ids[i] == marker_ID:
                        # rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], self.dim, self.matrix_coefficients, self.distortion_coefficients)
                        
                        if prev_rvec is not None:
                            _, rvec, tvec = cv2.solvePnP(self.obj_points, corners, self.matrix_coefficients, self.distortion_coefficients,rvec=prev_rvec.copy(), tvec=prev_tvec.copy(),useExtrinsicGuess=True,flags=cv2.SOLVEPNP_IPPE_SQUARE)
                        else:
                            _, rvec, tvec = cv2.solvePnP(self.obj_points, corners[i], self.matrix_coefficients, self.distortion_coefficients, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                        rvec, tvec = self.smooth_pose(rvec, tvec, prev_rvec, prev_tvec, alpha=0.3)
                        prev_rvec, prev_tvec = rvec.copy(), tvec.copy()
                        
                        # Build 4x4 pose matrix [R | t; 0 0 0 1]
                        R_mat, _ = cv2.Rodrigues(rvec)
                        rotation_matrix = np.eye(4, dtype=np.float32)
                        rotation_matrix[:3, :3] = R_mat  # Rotation part
                        rotation_matrix[:3, 3] = tvec.flatten()  # Translation part
                        rotation_matrix = np.dot(transformations[marker_IDs.index(marker_ID)], np.linalg.inv(rotation_matrix))

                        aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                        imgpts, jac = cv2.projectPoints(axis, rvec, tvec, self.matrix_coefficients,
                                                        self.distortion_coefficients)

                        cv2.drawFrameAxes(frame, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, length=0.1)
                        relativePoint = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
                        cv2.circle(frame, relativePoint, 2, (255, 255, 0))
           
            # Display the resulting frame
            cv2.imshow('frame', frame)
            key = cv2.waitKey(50) & 0xFF
            if key == ord('y'):
                return rotation_matrix
            elif key == ord('n'):
                return None
            
            time.sleep(0.1)


    # ── Static calibration ──────────────────────────────────────────────────────────────
    def static_calibration(self, marker_ID): 
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
                zipped = zip(ids, corners)
                ids, corners = zip(*(sorted(zipped)))
                axis = np.float32([[-0.01, -0.01, 0], [-0.01, 0.01, 0], [0.01, -0.01, 0], [0.01, 0.01, 0]]).reshape(-1, 3)

                prev_rvec, prev_tvec = None, None
                # Estimate pose of each marker
                for i in range(len(ids)):
                    if ids[i] == marker_ID:
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], self.dim, self.matrix_coefficients, self.distortion_coefficients)
                        
                        # Build 4x4 pose matrix [R | t; 0 0 0 1]
                        R_mat, _ = cv2.Rodrigues(rvec)
                        rotation_matrix = np.eye(4, dtype=np.float32)
                        rotation_matrix[:3, :3] = R_mat  # Rotation part
                        rotation_matrix[:3, 3] = tvec.flatten()  # Translation part
                        rotation_matrix = np.dot(transformations[marker_IDs.index(marker_ID)], np.linalg.inv(rotation_matrix))

                        aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                        imgpts, jac = cv2.projectPoints(axis, rvec, tvec, self.matrix_coefficients,
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
                    imgpts, jac = cv2.projectPoints(axis, rvec, tvec, self.matrix_coefficients,
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
#!/usr/bin/env python3

"""
░█▀▀░█▀█░█░░░▀█▀░█▀▄░█▀▄░█▀█░▀█▀░▀█▀░█▀█░█▀█
░█░░░█▀█░█░░░░█░░█▀▄░█▀▄░█▀█░░█░░░█░░█░█░█░█
░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀░░▀░▀░▀░▀░░▀░░▀▀▀░▀▀▀░▀░▀
"""

import numpy as np
import os
from utils.skeleton_tracker import SkeletonTracker
import pyrealsense2 as rs
from utils.marker_detector import MarkerDetector

# Parameters
script_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(script_dir, "data")
os.makedirs(data_dir, exist_ok=True)


# Function for saving rotation matrix on file
def write_rotation_matrix_to_file(file, mat):
    with open(file, 'w') as file:
        for i in range(4):
            file.write('\t'.join(map(str, mat[i, :])) + '\n')


# Function to ensure that average matrix is correctly defined (det(R)=1)
def correct_rotation_matrix(rotation_matrix):
    rot = rotation_matrix[:3, :3]
    U, _, Vt = np.linalg.svd(rot)
    R = np.dot(U, Vt)
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1   # flip last column
        R = np.dot(U, Vt)
    rotation_matrix[:3, :3] = R
    return rotation_matrix


# Main loop of marker detection and rotation matrix creation
def main():
    # Create pipeline and start config
    align = rs.align(rs.stream.color) # Allinea depth a color
    ctx = rs.context()
    devices = ctx.devices  # Query connected devices

    print("Press 'y' to accept the proposed reference system, otherwise press 'n' to skip")
    
    for device in devices:
        tracker = SkeletonTracker(device.get_info(rs.camera_info.serial_number), 1920, 1080, 30, False)
        mark = MarkerDetector(tracker)

        rotation_matrix = mark.simple_calibration(34)
        serial = device.get_info(rs.camera_info.serial_number)
        save_file = os.path.join(data_dir, f"calibration/pose_{serial}.txt")
        matrix = correct_rotation_matrix(rotation_matrix)
        write_rotation_matrix_to_file(save_file, matrix)

    print(f"Calibration ended correctly. Marker was detected by all the devices.")


# Entry point
if __name__ == '__main__':
    main()
    

    
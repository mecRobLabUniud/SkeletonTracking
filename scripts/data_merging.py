#!/usr/bin/env python3

"""
░█▀▄░█▀█░▀█▀░█▀█░░░█▄█░█▀▀░█▀▄░█▀▀░▀█▀░█▀█░█▀▀
░█░█░█▀█░░█░░█▀█░░░█░█░█▀▀░█▀▄░█░█░░█░░█░█░█░█
░▀▀░░▀░▀░░▀░░▀░▀░░░▀░▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀░▀░▀▀▀
Merging and re-shaping the skeleton.
Incoming data has mediapipe configuration:
0 - nose                9 - mouth (left)        18 - right pinky      27 - left ankle                     
1 - left eye (inner)    10 - mouth (right)      19 - left index       28 - right ankle            
2 - left eye            11 - left shoulder      20 - right index      29 - left heel      
3 - left eye (outer)    12 - right shoulder     21 - left thumb       30 - right heel            
4 - right eye (inner)   13 - left elbow         22 - right thumb      31 - left foot index         
5 - right eye           14 - right elbow        23 - left hip         32 - right foot index  
6 - right eye (outer)   15 - left wrist         24 - right hip               
7 - left ear            16 - right wrist        25 - left knee          
8 - right ear           17 - left pinky         26 - right knee     
"""

import sys
import numpy as np
from math import sin
from utils.kalman_filter import SimpleMerger, KalmanFilter3D, KalmanFilter6D, ImprovedKalmanFilter6D
from utils.data_transmitter import DataTransmitter
from utils.decorators import set_rate

# ─────────────────────────────────────────────────────────────────────────────
# Parameters
# ─────────────────────────────────────────────────────────────────────────────
running = True
n_devices = 0
skel_len = 0
kfs = None
cnt = 0


# ─────────────────────────────────────────────────────────────────────────────
# Re-shaping skeleton structure
# ─────────────────────────────────────────────────────────────────────────────
def reshape_structure(skeleton):
    new_skeleton = []
    head_markers = [wp for wp in skeleton[7:9] if not np.isnan(wp[0])]
    head = [[head_markers[i][j] for i in range(len(head_markers))] for j in range(3)]
    new_skeleton.append([sum(head[i])/len(head[i]) if len(head[i])>0 else np.nan for i in range(3)])
    for i in range(11, 17):
        new_skeleton.append(skeleton[i]) 

    left_hand_markers = [wp for wp in [skeleton[17], skeleton[19]] if not np.isnan(wp[0])]
    left_hand = [[left_hand_markers[i][j] for i in range(len(left_hand_markers))] for j in range(3)]
    new_skeleton.append([sum(left_hand[i])/len(left_hand[i]) if len(left_hand[i])>0 else np.nan for i in range(3)])
    right_hand_markers = [wp for wp in [skeleton[18], skeleton[20]] if not np.isnan(wp[0])]
    right_hand = [[right_hand_markers[i][j] for i in range(len(right_hand_markers))] for j in range(3)]
    new_skeleton.append([sum(right_hand[i])/len(right_hand[i]) if len(right_hand[i])>0 else np.nan for i in range(3)])
    
    upper_torso_markers = [wp for wp in skeleton[11:13] if not np.isnan(wp[0])]
    lower_torso_markers = [wp for wp in skeleton[23:25] if not np.isnan(wp[0])]
    upper_torso = [[upper_torso_markers[i][j] for i in range(len(upper_torso_markers))] for j in range(3)]
    lower_torso = [[lower_torso_markers[i][j] for i in range(len(lower_torso_markers))] for j in range(3)]
    new_skeleton.append([sum(upper_torso[i])/len(upper_torso[i]) if len(upper_torso[i])>0 else np.nan for i in range(3)])
    new_skeleton.append([sum(lower_torso[i])/len(lower_torso[i]) if len(lower_torso[i])>0 else np.nan for i in range(3)])
    for i in range(23, 33):
        new_skeleton.append(skeleton[i]) 

    new_skeleton = np.asanyarray(new_skeleton)
    return new_skeleton


# ─────────────────────────────────────────────────────────────────────────────
# Merging
# ─────────────────────────────────────────────────────────────────────────────
@set_rate(60)
def merging(dtrs, dts):
    global cnt
    skeletons = []
    confidences = []
    for dtr in dtrs:
        skeleton, confidence = dtr.receive_data()
        # print(confidence)
        if skeleton is None or confidence is None or confidence is None:
            skeletons.append(None)
            confidences.append(None)
        else:
            skeletons.append(skeleton)
            confidences.append(confidence)

    
    
    merged_skeleton = []
    for i in range(skel_len):
        skeleton_marker = [skeleton[i] for skeleton in skeletons if not skeleton==None]
        confidence_marker = [confidence[i] for confidence in confidences if not confidence==None]
        merged_skeleton.append(kfs[i].step(skeleton_marker, confidence_marker).tolist())
    
    cnt += 0.01

    reshaped_skeleton = np.asanyarray([[0.35, -0.4 + 0.1*sin(cnt), 0.5]]) # reshape_structure(merged_skeleton)    
    merged_confidence = np.ones(skel_len).astype(np.float32)

    dts.send_data(reshaped_skeleton, merged_confidence)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point 
# ─────────────────────────────────────────────────────────────────────────────
def main():
    global n_devices, skel_len, kfs
    arg1 = sys.argv[1] if len(sys.argv) > 1 else None
    if arg1 is None:
        raise ValueError("No argument provided. Enter the number of cameras")   
    else:
        try:
            n_devices = int(arg1)  
        except:
            raise ValueError(f"Wrong argument: {arg1}")
        
    dtrs = [DataTransmitter("receiver", n, "SINGLE_CAMERA") for n in range(n_devices)]
    dts = DataTransmitter("sender", 10, "MERGED")
    print("Merging started correctly\n")

    skeleton, _ = dtrs[0].receive_data()
    skel_len = len(skeleton)
    kfs = [KalmanFilter6D() for _ in range(skel_len)]

    # Main loop
    while running:
        merging(dtrs, dts)

    for dtr in dtrs:
        dtr.shutdown()
    dts.shutdown()
        

if __name__ == "__main__":
    main()
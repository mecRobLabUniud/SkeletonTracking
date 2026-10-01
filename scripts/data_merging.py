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
from utils.kalman_filter import KalmanFilter6D
from utils.data_transmitter import DataTransmitter
from utils.decorators import set_rate
from dataclasses import dataclass


@dataclass
class Traj:
    p: np.ndarray  # dim x n
    v: np.ndarray  # dim x n
    a: np.ndarray  # dim x n


def quintic_poly_traj(x0, x1, T, dt, n):
    x0 = np.asarray(x0, dtype=float).reshape(-1)
    x1 = np.asarray(x1, dtype=float).reshape(-1)
    d = x0.size
    dx = x1 - x0

    p = np.empty((d, n))
    v = np.empty((d, n))
    a = np.empty((d, n))

    for k in range(n):
        t = min(k * dt, T)
        s = t / T
        s2 = s * s
        s3 = s2 * s
        s4 = s3 * s
        s5 = s4 * s
        h = 10 * s3 - 15 * s4 + 6 * s5
        dh = (30 * s2 - 60 * s3 + 30 * s4) / T
        ddh = (60 * s - 180 * s2 + 120 * s3) / (T * T)
        p[:, k] = x0 + dx * h
        v[:, k] = dx * dh
        a[:, k] = dx * ddh

    return Traj(p, v, a)

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
    
    cnt += 1


    def lround(x):
        # C++ std::lround rounds half away from zero (Python's round() is banker's rounding)
        return int(np.floor(x + 0.5)) if x >= 0 else int(np.ceil(x - 0.5))


    freq = 60
    dt = 1.0 / freq
    time_final = 5.0
    time_experiment = 30.0
    N = lround(time_final / dt) + 1  # 1001
    Nc = lround(time_experiment / dt) + 1
    M = (N - 1) * 3 + 1

    
    # ------------------------------------------------------ Human trajectory
    t_move, t_pause = 1.75, 1.5
    Nm = lround(t_move * freq) + 1                 # 251
    Nh = lround((2 * t_move + t_pause) * freq) + 1  # 1001

    hA = np.array([0.8, 0.8, 0.3])
    hB = np.array([0.4, 0.4, 0.3])

    hf = quintic_poly_traj(hA, hB, t_move, dt, Nm)  # A -> B
    hs = quintic_poly_traj(hB, hA, t_move, dt, Nm)  # B -> A

    p_unit = np.tile(hA.reshape(3, 1), (1, Nh))     # every column = hA
    v_unit = np.zeros((3, Nh))

    m = Nm - 1  # 250: last index of first move / first of second
    p_unit[:, 0:Nm] = hf.p
    v_unit[:, 0:Nm] = hf.v
    p_unit[:, m:m + Nm] = hs.p   # overwrites the shared sample at index m
    v_unit[:, m:m + Nm] = hs.v

    p_int = np.zeros((3, Nc))
    v_int = np.zeros((3, Nc))

    reps = lround(time_experiment / time_final)
    for j in range(reps):
        start = j * int(freq * time_final)
        p_int[:, start:start + Nh] = p_unit
        v_int[:, start:start + Nh] = v_unit

    if cnt >= Nc:
        cnt = 0

    # p1 = [0.5, 0.2 - 0.1*sin(cnt), 0.5]
    p1 = p_int[:, cnt]
    # p1 = [0.8 - 0.4*abs(sin(cnt)), 0.8 - 0.4*abs(sin(cnt)), 0.3]
    p2 = [p1[0], p1[1], p1[2]-0.01]
    reshaped_skeleton = np.asanyarray([p1, p2]) # reshape_structure(merged_skeleton)    
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
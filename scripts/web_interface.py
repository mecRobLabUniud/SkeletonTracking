#!/usr/bin/env python3

"""
░█░█░█▀▀░█▀▄░░░▀█▀░█▀█░▀█▀░█▀▀░█▀▄░█▀▀░█▀█░█▀▀░█▀▀
░█▄█░█▀▀░█▀▄░░░░█░░█░█░░█░░█▀▀░█▀▄░█▀▀░█▀█░█░░░█▀▀
░▀░▀░▀▀▀░▀▀░░░░▀▀▀░▀░▀░░▀░░▀▀▀░▀░▀░▀░░░▀░▀░▀▀▀░▀▀▀

User interface for the rendering of the 3D reconstruction
of the skeleton, according to the new keypoint strcture.
Incoming data has custom configuration:
0 - head                11 - left hip
1 - left shoulder       12 - right hip
2 - right shoulder      13 - left knee
3 - left elbow          14 - right knee
4 - right elbow         15 - left ankle
5 - left wrist          16 - right ankle
6 - right wrist         17 - left heel               
7 - left hand           18 - right heel         
8 - right hand          19 - left foot
9 - upper torso         20 - right foot
10 - lower torso    
"""

import webbrowser
import threading
import numpy as np
import sys
from flask import Flask, render_template
from flask_socketio import SocketIO
from utils.data_transmitter import DataTransmitter
from utils.decorators import set_rate

# ─────────────────────────────────────────────────────────────────────────────
# Parameters 
# ─────────────────────────────────────────────────────────────────────────────

app = Flask(__name__, 
            template_folder='flask_utils', 
            static_folder='flask_utils', 
            static_url_path='')
socketio = SocketIO(app, cors_allowed_origins="*")
dtrs = None
use_robot = False


# ─────────────────────────────────────────────────────────────────────────────
# Rula thread 
# ─────────────────────────────────────────────────────────────────────────────
@set_rate(60)
def send_rula_score():
    try:
        score = dtrs[n_devices+1].receive_rula_score()
        socketio.emit("update_rula", score)
        
    except Exception as e:
        print(f"RULA thread error: {e}")

def rula_thread():
    while True:
        send_rula_score()


# ─────────────────────────────────────────────────────────────────────────────
# Skeleton thread 
# ─────────────────────────────────────────────────────────────────────────────
@set_rate(60)
def send_data():
    try:
        merged_skeleton = dtrs[n_devices].receive_data()[0]
        
        x = [pnt[0] if not np.isnan(pnt[0]) else None for pnt in merged_skeleton]
        y = [pnt[1] if not np.isnan(pnt[1]) else None for pnt in merged_skeleton]
        z = [pnt[2] if not np.isnan(pnt[2]) else None for pnt in merged_skeleton]

        if use_robot:
            robot_p = dtrs[n_devices+2].receive_data()[0]
            robot_q = dtrs[n_devices+2].receive_data()[1]
            caps_radius = dtrs[n_devices+2].receive_data()[2]

            x_robot = [pnt[0] if not np.isnan(pnt[0]) else None for pnt in robot_p]
            y_robot = [pnt[1] if not np.isnan(pnt[1]) else None for pnt in robot_p]
            z_robot = [pnt[2] if not np.isnan(pnt[2]) else None for pnt in robot_p]
            q_robot = [q if not np.isnan(q) else None for q in robot_q]
            radius = [radius if not np.isnan(radius) else None for radius in caps_radius]

            c_h = dtrs[n_devices+3].receive_data()[0]
            c_r = dtrs[n_devices+3].receive_data()[1]

            p_real = dtrs[n_devices+4].receive_data()[0]
            p_r = dtrs[n_devices+4].receive_data()[1]

            msg = {"x": x, "y": y, "z": z, "x_robot": x_robot, "y_robot": y_robot, "z_robot": z_robot,
             "q_robot": q_robot, "radius": radius, "c_h": c_h, "c_r": c_r, "p_real": p_real, "p_r": p_r}
        else:
            msg = {"x": x, "y": y, "z": z}

        socketio.emit("update_plot", msg)
        
    except Exception as e:
        print(f"Skeleton thread error: {e}")

def skeleton_thread():
    while True:
        send_data()


# ─────────────────────────────────────────────────────────────────────────────
# Frame thread
# ─────────────────────────────────────────────────────────────────────────────
@set_rate(60)
def send_frames():
    try:
        frames = [dtr.receive_frames() for dtr in dtrs[0:n_devices]]
        for n, frame in enumerate(frames):
            socketio.emit(f"update_stream{n+1}", {"frame": frame})
    except Exception as e:
        print(f"Image thread error: {e}")

def frame_thread():
    while True:
        send_frames()
        

# ─────────────────────────────────────────────────────────────────────────────
# Web interface route
# ─────────────────────────────────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("index.html")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point 
# ─────────────────────────────────────────────────────────────────────────────
def main():
    global dtrs, n_devices, use_robot
    arg1 = sys.argv[1] if len(sys.argv) > 1 else None
    arg2 = sys.argv[2] if len(sys.argv) > 2 else None
    if arg1 is None:
        raise ValueError("No argument provided. Enter the number of cameras")   
    else:
        try:
            n_devices = int(arg1)  
        except:
            raise ValueError(f"Wrong argument: {arg1}")
        try:
            if arg2 == "--robot":
                use_robot = True
        except:
            raise ValueError(f"Wrong argument: {arg2}")

    dtrs = [DataTransmitter("receiver", n, "SINGLE_CAMERA") for n in range(n_devices)]
    dtrs.append(DataTransmitter("receiver", 10, "MERGED"))
    dtrs.append(DataTransmitter("receiver", 11, "RULA"))
    if use_robot: 
        dtrs.append(DataTransmitter("receiver", 12, "ROBOT"))
        dtrs.append(DataTransmitter("receiver", 13, "DISTANCE"))
        dtrs.append(DataTransmitter("receiver", 14, "TRAJDATA"))

    threading.Thread(target=skeleton_thread, daemon=True).start()              
    threading.Thread(target=frame_thread, daemon=True).start()
    threading.Thread(target=rula_thread, daemon=True).start()
    webbrowser.open_new('http://127.0.0.1:5000/')
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)
    

if __name__ == "__main__":
    main()
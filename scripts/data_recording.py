#!/usr/bin/env python3

"""
░█▀▄░█▀█░▀█▀░█▀█░░░█▀▄░█▀▀░█▀▀░█▀█░█▀▄░█▀▄░▀█▀░█▀█░█▀▀
░█░█░█▀█░░█░░█▀█░░░█▀▄░█▀▀░█░░░█░█░█▀▄░█░█░░█░░█░█░█░█
░▀▀░░▀░▀░░▀░░▀░▀░░░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀░▀░▀▀░░▀▀▀░▀░▀░▀▀▀
"""

import sys
import zmq
import time
import numpy as np
import cv2
import os
import json
import signal
import struct
import threading
import multiprocessing.resource_tracker as rt
from multiprocessing import shared_memory
from utils.data_transmitter import DataTransmitter
from utils.video_recorder import VideoRecorder
from utils.decorators import chronometer, set_rate

# ─────────────────────────────────────────────────────────────────────────────
# Parameters
# ─────────────────────────────────────────────────────────────────────────────
in_port = 6000
out_port = 6000
topic = "SKEL"
running = True
skel_len = 17
H, W, C = 480, 848, 3
FRAME_BYTES = H * W * C

stream_cnt = 0
n_devices = 0
frame_id = 0
paused = False
reset = False
script_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(script_dir, "data")
os.makedirs(data_dir, exist_ok=True)
t0 = time.time()


# ─────────────────────────────────────────────────────────────────────────────
# Pause/resume logic
# ─────────────────────────────────────────────────────────────────────────────
def listen_for_input():
    """Listen for keyboard input in a separate thread."""
    global paused
    while True:
        key = input()
        if key.strip().lower() == '' and paused:
            paused = False
            print("\n▶  Loop RESUMED")
        elif key.strip().lower() == '' and not paused:
            paused = True
            print("\n⏸  Loop PAUSED")
 

# ─────────────────────────────────────────────────────────────────────────────
# Recording
# ─────────────────────────────────────────────────────────────────────────────
@set_rate(60)
def record_data(dtrs, skeleton_data_writers, color_writers):
    for dtr, skeleton_data_writer, color_writer in zip(dtrs, skeleton_data_writers, color_writers):
        skeleton_data_packed = dtr.receive_packed_msg()

        with open(skeleton_data_writer, "a") as file:
            file.write(f"{time.time()-t0}; {skeleton_data_packed}\n")

        raw_frame = dtr.receive_raw_frames()
        color_writer.write(raw_frame)
        print("Acquiring...", end="\r")


# ─────────────────────────────────────────────────────────────────────────────
# Streaming
# ─────────────────────────────────────────────────────────────────────────────
@set_rate(60)
def stream_data(dtss, skeleton_data_readers, color_readers):
    global stream_cnt, reset
    for dts, skeleton_data_reader, color_reader in zip(dtss, skeleton_data_readers, color_readers):
        with open(skeleton_data_reader, "r") as file:
            lines = file.readlines()
            if stream_cnt >= len(lines):
                reset = True
                break
            skeleton_data_packed = lines[stream_cnt]

            time, _, skeleton_packed, confidence_packed = skeleton_data_packed.split("; ", 3)
            skeleton = np.array(json.loads(skeleton_packed))
            confidence = np.array(json.loads(confidence_packed))
            dts.send_data(skeleton, confidence)

        ret, frame = color_reader.read()
        if ret:
            dts.send_frames(frame)
    stream_cnt += 1


# ─────────────────────────────────────────────────────────────────────────────
# Entry point 
# ─────────────────────────────────────────────────────────────────────────────
def main():
    global n_devices, stream_cnt, reset
    
    arg1 = sys.argv[1] if len(sys.argv) > 1 else None
    arg2 = sys.argv[2] if len(sys.argv) > 2 else None
    arg3 = sys.argv[3] if len(sys.argv) > 3 else None

    # Start input listener in background thread
    # input_thread = threading.Thread(target=listen_for_input, daemon=True)
    # input_thread.start()

    # Clear shutdown logic
    def signal_handler(sig, frame):
        global running
        running = False
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        skeleton_data_dir = os.path.join(data_dir, f"skeleton_data")
        media_dir = os.path.join(data_dir, f"media")
        n_test_skeleton_data = max([int(directory[4:]) for directory in list(os.walk(skeleton_data_dir))[0][1]]) if not list(os.walk(skeleton_data_dir))[0][1] == [] else 0
        n_test_media = max([int(directory[4:]) for directory in list(os.walk(media_dir))[0][1]]) if not list(os.walk(media_dir))[0][1] == [] else 0
        n_test = max(n_test_skeleton_data, n_test_media)
    except:
        skeleton_data_dir = os.path.join(data_dir, f"skeleton_data")
        n_test = max([int(directory[4:]) for directory in list(os.walk(skeleton_data_dir))[0][1]]) if not list(os.walk(skeleton_data_dir))[0][1] == [] else 0

    if arg1 is None:
        raise ValueError("No argument provided. Enter the number of cameras")   
    else:
        try:
            n_devices = int(arg1)  
        except:
            raise ValueError(f"Wrong argument: {arg1}")

    if arg2 is None:
        raise ValueError("No argument provided. Use --record or --stream.")

    if arg2 in ["--record", "-r"]:
        skeleton_data_test_dir = os.path.join(skeleton_data_dir, f"test{n_test+1}")
        media_test_dir = os.path.join(media_dir, f"test{n_test+1}")
        os.makedirs(skeleton_data_test_dir, exist_ok=True)  
        os.makedirs(media_test_dir, exist_ok=True)  

        dtrs = [DataTransmitter("receiver", n, "SINGLE_CAMERA") for n in range(n_devices)]
        skeleton_data_writers = [os.path.join(skeleton_data_test_dir, f"skeleton_{n}.txt") for n in range(int(n_devices))]
        [open(skeleton_data_writer, "w") for skeleton_data_writer in skeleton_data_writers]
        color_writers = [VideoRecorder(os.path.join(media_test_dir, f"color_{n}.avi"), "XVID", 60, (848, 480), is_color=True) for n in range(n_devices)]
        print("Recording mode enabled. Press Ctrl+C to stop.")
        while running:
            record_data(dtrs, skeleton_data_writers, color_writers)
        for dtr, color_writer in zip(dtrs, color_writers):
            dtr.shutdown()
            color_writer.release()

    elif arg2 in ["--stream", "-s"]:
        if arg3 is None:
            raise ValueError("No argument provided. Enter the number of test to stream")   
        else:
            try:
                n_test = int(arg3)  
            except:
                raise ValueError(f"Wrong argument: {arg3}")

        skeleton_data_test_dir = os.path.join(skeleton_data_dir, f"test{n_test}")
        media_test_dir = os.path.join(media_dir, f"test{n_test}")

        dtss = [DataTransmitter("sender", n, "SINGLE_CAMERA") for n in range(n_devices)]
        skeleton_data_readers = [os.path.join(skeleton_data_test_dir, f"skeleton_{n}.txt") for n in range(int(n_devices))]
        color_readers = [cv2.VideoCapture(os.path.join(media_test_dir, f"color_{n}.avi")) for n in range(n_devices)]
        print("Streaming mode enabled. Press Ctrl+C to stop.")
        try:
            while running:
                if not paused:
                    stream_data(dtss, skeleton_data_readers, color_readers)
                    if reset:
                        reset = False
                        color_readers = [cv2.VideoCapture(os.path.join(media_test_dir, f"color_{n}.avi")) for n in range(n_devices)]
                        stream_cnt = 0
                else:
                    time.sleep(0.016)                
        finally:
            for dts in dtss:
                dts.shutdown()

    else:
        raise ValueError(f"Unknown argument: {arg2}")


if __name__ == "__main__":
    main()
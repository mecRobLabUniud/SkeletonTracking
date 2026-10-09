#!/usr/bin/env python3

"""
░█▀▄░█▀█░▀█▀░█▀█░░░▀█▀░█▀▄░█▀█░█▀█░█▀▀░█▄█░▀█▀░▀█▀░▀█▀░█▀▀░█▀▄
░█░█░█▀█░░█░░█▀█░░░░█░░█▀▄░█▀█░█░█░▀▀█░█░█░░█░░░█░░░█░░█▀▀░█▀▄
░▀▀░░▀░▀░░▀░░▀░▀░░░░▀░░▀░▀░▀░▀░▀░▀░▀▀▀░▀░▀░▀▀▀░░▀░░░▀░░▀▀▀░▀░▀
"""

import cv2
import zmq
import json
import sys
import warnings
import time
import base64
import numpy as np
import multiprocessing.resource_tracker as rt
from multiprocessing import shared_memory
from utils.decorators import requires


# ── Parameters ───────────────────────────────────────────────────────────────
H, W, C = 480, 848, 3


# ─────────────────────────────────────────────────────────────────────────────
# Manage a named shared-memory segment, either creating or attaching to it
# ─────────────────────────────────────────────────────────────────────────────
class SharedMemoryManager:
    # ─────────────────────────────────────────────────────────────────────────
    # Create or attach to the shared-memory segment
    # ─────────────────────────────────────────────────────────────────────────
    def __init__(self, name: str, size: int, create: bool):
        self.name = name
        self.size = size
        self.create = create
        self._shm: shared_memory.SharedMemory | None = None
        self._open()


    # ── Shutdown block ───────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # Close the local handle to the shared memory
    # ─────────────────────────────────────────────────────────────────────────
    def close(self):
        if self._shm is not None:
            try:
                self._shm.close()
            except (UserWarning, Exception):
                pass
            self._shm = None

    # ─────────────────────────────────────────────────────────────────────────
    # Release the shared-memory segment from the system
    # ─────────────────────────────────────────────────────────────────────────
    def unlink(self):
        if not self.create:
            return
        try:
            # ── Re-open temporarily just to unlink the segment ───────────────
            tmp = shared_memory.SharedMemory(name=self.name, create=False, size=self.size)
            tmp.close()
            tmp.unlink()
        except (KeyError, Exception, FileNotFoundError):
            pass

    # ─────────────────────────────────────────────────────────────────────────
    # Close and release the shared-memory segment
    # ─────────────────────────────────────────────────────────────────────────
    def shutdown(self):
        self.close()
        self.unlink()


    # ── Destructor ───────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # Shut down the segment when the manager is garbage collected
    # ─────────────────────────────────────────────────────────────────────────
    def __del__(self):
        try:
            self.shutdown()
        except Exception:
            pass


    # ── Internals ────────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # Open the segment, creating or attaching according to the flag
    # ─────────────────────────────────────────────────────────────────────────
    def _open(self):
        if self.create:
            self._shm = self._create_or_replace()
        else:
            self._shm = self._attach()

    # ─────────────────────────────────────────────────────────────────────────
    # Create the segment, replacing a stale one if it already exists
    # ─────────────────────────────────────────────────────────────────────────
    def _create_or_replace(self) -> shared_memory.SharedMemory:
        try:
            return shared_memory.SharedMemory(name=self.name, create=True, size=self.size)
        except FileExistsError:
            try:
                stale = shared_memory.SharedMemory(name=self.name, create=False, size=0)
                stale.close()
                stale.unlink()
            except Exception:
                pass
            return shared_memory.SharedMemory(name=self.name, create=True, size=self.size)

    # ─────────────────────────────────────────────────────────────────────────
    # Attach to the segment, waiting until it becomes available
    # ─────────────────────────────────────────────────────────────────────────
    def _attach(self) -> shared_memory.SharedMemory:
        attached = False
        while not attached:
            try:
                shm = shared_memory.SharedMemory(name=self.name, create=False, size=self.size)
                self._suppress_tracker(shm)
                attached = True
            except (ValueError, FileNotFoundError):
                time.sleep(0.01)
        return shm

    # ─────────────────────────────────────────────────────────────────────────
    # Prevent the resource tracker from warning about the attached segment
    # ─────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _suppress_tracker(shm: shared_memory.SharedMemory):
        if sys.version_info >= (3, 9):
            try:
                rt.unregister(f"/{shm.name}", "shared_memory")
            except Exception:
                pass
        else:
            warnings.filterwarnings(
                "ignore",
                category=UserWarning,
                message=".*resource_tracker.*leaked.*shared_memory.*",
                module="multiprocessing.resource_tracker",
            )

    
    # ── Properties ───────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # Raw buffer backing the shared-memory segment
    # ─────────────────────────────────────────────────────────────────────────
    @property
    def buf(self):
        if self._shm is None:
            raise RuntimeError(f"SharedMemory '{self.name}' is not open.")
        return self._shm.buf


# ─────────────────────────────────────────────────────────────────────────────
# Send and receive skeleton, frame and RULA messages over ZeroMQ and shared
# memory
# ─────────────────────────────────────────────────────────────────────────────
class DataTransmitter:
    # ─────────────────────────────────────────────────────────────────────────
    # Set up the ZeroMQ socket and shared memory for the requested mode
    # ─────────────────────────────────────────────────────────────────────────
    def __init__(self, mode: str, device_id: int, topic: str, port: int = 6000):
        self.mode = mode
        self.device_id = device_id
        self.port = port + device_id
        self.topic = topic
        self.nbytes = H * W * C
        self.socket = None
        self.shm: SharedMemoryManager | None = None

        if self.mode == "sender":
            self.setup_zmq_sender()
            self.setup_shm_sender()
            self.send_frames = self._send_frames
            self.send_data = self._send_data
            self.send_rula_score = self._send_rula_score
        elif self.mode == "receiver":
            self.setup_zmq_receiver()
            self.setup_shm_receiver()
            self.receive_raw_frames = self._receive_raw_frames
            self.receive_packed_msg = self._receive_packed_msg
            self.receive_frames = self._receive_frames
            self.receive_data = self._receive_data
            self.receive_rula_score = self._receive_rula_score
        else:
            raise ValueError(f"Unknown argument: {self.mode}")


    # ── Destructor ───────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # Shut down the transmitter when it is garbage collected
    # ─────────────────────────────────────────────────────────────────────────
    def __del__(self):
        try:
            self.shutdown()
        except Exception:
            pass


    # ── ZeroMQ setup ─────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # Bind a publish socket for the sender mode
    # ─────────────────────────────────────────────────────────────────────────
    def setup_zmq_sender(self):
        try:
            socket = zmq.Context.instance().socket(zmq.PUB)
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.SNDHWM, 1)
            socket.bind(f"tcp://*:{self.port}")
            self.socket = socket
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────────
    # Connect a subscribe socket for the receiver mode
    # ─────────────────────────────────────────────────────────────────────────
    def setup_zmq_receiver(self):
        socket = zmq.Context.instance().socket(zmq.SUB)
        socket.setsockopt(zmq.CONFLATE, 1)
        socket.setsockopt_string(zmq.SUBSCRIBE, f"{self.topic}_{self.device_id}")
        socket.connect(f"tcp://localhost:{self.port}")
        self.socket = socket


    # ── Shared memory setup ──────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # Create the shared-memory segment for the sender mode
    # ─────────────────────────────────────────────────────────────────────────
    def setup_shm_sender(self):
        self.shm = SharedMemoryManager(name=f"shared_image{self.device_id}", size=self.nbytes, create=True, )

    # ─────────────────────────────────────────────────────────────────────────
    # Attach to the shared-memory segment for the receiver mode
    # ─────────────────────────────────────────────────────────────────────────
    def setup_shm_receiver(self):
        self.shm = SharedMemoryManager(name=f"shared_image{self.device_id}", size=self.nbytes, create=False, )


    # ── Helpers ──────────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # Encode an image as a base64 JPEG data URL
    # ─────────────────────────────────────────────────────────────────────────
    @staticmethod
    def cv2_to_b64(img):
        is_success, buffer = cv2.imencode(
            ".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90]
        )
        if not is_success:
            return None
        encoded = base64.b64encode(buffer).decode("utf-8")
        return "data:image/jpeg;base64," + encoded


    # ── Send block ───────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # Copy a frame into shared memory
    # ─────────────────────────────────────────────────────────────────────────
    @requires("sender")
    def _send_frames(self, frame: np.ndarray):
        buf = np.ndarray(frame.shape, dtype=frame.dtype, buffer=self.shm.buf)
        buf[:] = frame[:]

    # ─────────────────────────────────────────────────────────────────────────
    # Publish the given arrays as one JSON message
    # ─────────────────────────────────────────────────────────────────────────
    @requires("sender")
    def _send_data(self, *array: np.ndarray):
        msg = f"{self.topic}_{self.device_id}"
        for elem in array:
            msg += f"; {json.dumps(elem.tolist())}"
        self.socket.send_string(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Publish the RULA score as a JSON message
    # ─────────────────────────────────────────────────────────────────────────
    @requires("sender")
    def _send_rula_score(self, score: list):
        msg = (f"{self.topic}_{self.device_id}; " f"{json.dumps(score)}")
        self.socket.send_string(msg)


    # ── Receive block ────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # Copy the latest frame out of shared memory
    # ─────────────────────────────────────────────────────────────────────────
    @requires("receiver")
    def _receive_raw_frames(self) -> np.ndarray:
        return np.ndarray((H, W, C), dtype=np.uint8, buffer=self.shm.buf).copy()

    # ─────────────────────────────────────────────────────────────────────────
    # Receive the next raw message from the socket
    # ─────────────────────────────────────────────────────────────────────────
    @requires("receiver")
    def _receive_packed_msg(self) -> str:
        return self.socket.recv_string()

    # ─────────────────────────────────────────────────────────────────────────
    # Receive the latest frame as a base64 data URL
    # ─────────────────────────────────────────────────────────────────────────
    @requires("receiver")
    def _receive_frames(self) -> str:
        return self.cv2_to_b64(self.receive_raw_frames())

    # ─────────────────────────────────────────────────────────────────────────
    # Receive and decode the JSON arrays of a message
    # ─────────────────────────────────────────────────────────────────────────
    @requires("receiver")
    def _receive_data(self):
        packed = self.receive_packed_msg()
        _, *packed_data = packed.split("; ")
        return [json.loads(packed) for packed in packed_data]
    
    # ─────────────────────────────────────────────────────────────────────────
    # Receive and decode the RULA score
    # ─────────────────────────────────────────────────────────────────────────
    @requires("receiver")
    def _receive_rula_score(self):
        packed = self.receive_packed_msg()
        _, score_packed = packed.split("; ", 1)
        score = json.loads(score_packed)[0]
        return score


    # ── Shutdown ─────────────────────────────────────────────────────────────
    # ─────────────────────────────────────────────────────────────────────────
    # Close the socket and release the shared memory
    # ─────────────────────────────────────────────────────────────────────────
    def shutdown(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        if self.shm is not None:
            self.shm.shutdown()
            self.shm = None

#!/usr/bin/env python3

"""
░█▀▀░▀█▀░█░░░▀█▀░█▀▀░█▀▄░█▀▀
░█▀▀░░█░░█░░░░█░░█▀▀░█▀▄░▀▀█
░▀░░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░▀▀▀

Script for filtering and smoothing 3D keypoints of human skeletons, using One Euro Filter 
for temporal smoothing and occlusion handling.
Includes a robust depth reading function that computes the median depth in a neighborhood 
to handle noise and missing data from the RealSense camera.
"""

import time
import math
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# One Euro Filter for smoothing a 1D signal, adapted to 3D keypoints in
# Keypoints3DSmoother
# ─────────────────────────────────────────────────────────────────────────────
class OneEuroFilter:
    # ─────────────────────────────────────────────────────────────────────────
    # Initialize the filter with its smoothing parameters
    # ─────────────────────────────────────────────────────────────────────────
    def __init__(self, t0, x0, dx0=0.0, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_prev = float(x0)
        self.dx_prev = float(dx0)
        self.t_prev = float(t0)

    # ─────────────────────────────────────────────────────────────────────────
    # Compute the smoothing factor for a time step and cutoff frequency
    # ─────────────────────────────────────────────────────────────────────────
    def smoothing_factor(self, t_e, cutoff):
        r = 2.0 * math.pi * cutoff * t_e
        return r / (r + 1.0)

    # ─────────────────────────────────────────────────────────────────────────
    # Apply exponential smoothing between the new and previous values
    # ─────────────────────────────────────────────────────────────────────────
    def exponential_smoothing(self, alpha, x, x_prev):
        return alpha * x + (1.0 - alpha) * x_prev

    # ─────────────────────────────────────────────────────────────────────────
    # Update the filter with a new sample and return the smoothed value
    # ─────────────────────────────────────────────────────────────────────────
    def __call__(self, t, x):
        # ── Time step ────────────────────────────────────────────────────────
        t_e = t - self.t_prev
        if t_e <= 0.0:
            return self.x_prev
        # ── Smoothed velocity estimate ───────────────────────────────────────
        a_d = self.smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = self.exponential_smoothing(a_d, dx, self.dx_prev)
        # ── Cutoff adapted to the measured speed ─────────────────────────────
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        # ── Filtered position ────────────────────────────────────────────────
        a = self.smoothing_factor(t_e, cutoff)
        x_hat = self.exponential_smoothing(a, x, self.x_prev)
        # ── State update ─────────────────────────────────────────────────────
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat


# ─────────────────────────────────────────────────────────────────────────────
# Smooth 3D keypoints with One Euro Filters, holding each last valid position
# for a short time during occlusions
# ─────────────────────────────────────────────────────────────────────────────
class Keypoints3DSmoother:
    # ─────────────────────────────────────────────────────────────────────────
    # Initialize the filter parameters and data structures
    # ─────────────────────────────────────────────────────────────────────────
    def __init__(self, num_kpts=17, min_cutoff=0.1, beta=1.0):
        self.num_kpts = num_kpts
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.t0 = time.monotonic()
        self.initialized = False
        self.filters = []
        self.last_valid = np.full((num_kpts, 3), np.nan, dtype=np.float32)
        self.last_valid_time = np.zeros(num_kpts, dtype=np.float64)

    # ─────────────────────────────────────────────────────────────────────────
    # Apply the filters to a new frame of keypoints and confidence values
    # ─────────────────────────────────────────────────────────────────────────
    def update(self, xyz, conf, conf_thr):
        # ── Relative time ────────────────────────────────────────────────────
        t = time.monotonic() - self.t0
        # ── Lazy filter initialization on the first valid frame ──────────────
        if not self.initialized:
            for i in range(self.num_kpts):
                x0 = float(xyz[i, 0]) if np.isfinite(xyz[i, 0]) else 0.0
                y0 = float(xyz[i, 1]) if np.isfinite(xyz[i, 1]) else 0.0
                z0 = float(xyz[i, 2]) if np.isfinite(xyz[i, 2]) else 0.0
                self.filters.append((
                    OneEuroFilter(t, x0, min_cutoff=self.min_cutoff, beta=self.beta),
                    OneEuroFilter(t, y0, min_cutoff=self.min_cutoff, beta=self.beta),
                    OneEuroFilter(t, z0, min_cutoff=self.min_cutoff, beta=self.beta),
                ))
            self.initialized = True

        out = np.copy(xyz).astype(np.float32)
        for i in range(self.num_kpts):
            # ── Keypoint validity from confidence and finiteness ─────────────
            valid = (conf[i] >= conf_thr) and np.all(np.isfinite(xyz[i]))
            
            if not valid:
                # ── Occlusion handling with a 0.5 s validity window ─────────
                if np.all(np.isfinite(self.last_valid[i])) and (t - self.last_valid_time[i] < 0.5):
                    out[i] = self.last_valid[i]
                else:
                    out[i] = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
                continue
            # ── Filter each axis independently ───────────────────────────────
            fx, fy, fz = self.filters[i]
            out[i, 0] = fx(t, float(xyz[i, 0]))
            out[i, 1] = fy(t, float(xyz[i, 1]))
            out[i, 2] = fz(t, float(xyz[i, 2]))
            self.last_valid[i] = out[i]
            self.last_valid_time[i] = t
        return out
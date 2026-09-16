#!/usr/bin/env python3
 
"""
░█░█░█▀█░█░░░█▄█░█▀█░█▀█░░░█▀▀░▀█▀░█░░░▀█▀░█▀▀░█▀▄
░█▀▄░█▀█░█░░░█░█░█▀█░█░█░░░█▀▀░░█░░█░░░░█░░█▀▀░█▀▄
░▀░▀░▀░▀░▀▀▀░▀░▀░▀░▀░▀▀▀░░░▀░░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀

Merges two or more noisy arrays into a single optimal estimate
using a 6-D Kalman filter with multi-sensor sequential updates.
"""

import numpy as np


class KalmanFilter6D:
    def __init__(self):
        self.conf_thresh = 0.5
        self.maha_thresh = 9.0
        self.n   = 6
        self.dt = 50.0
        self.s_k = np.zeros(self.n)
        self.p_k = np.eye(self.n) * 0.1
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3])
        self.H_k = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])
        self.F_k = np.array([[1, 0, 0, self.dt, 0,  0 ],
                            [0, 1, 0, 0,  self.dt, 0 ],
                            [0, 0, 1, 0,  0,  self.dt],
                            [0, 0, 0, 1,  0,  0 ],
                            [0, 0, 0, 0,  1,  0 ],
                            [0, 0, 0, 0,  0,  1 ]])

    def predict(self):
        self.s_k = self.F_k.dot(self.s_k)
        self.p_k = self.F_k.dot(self.p_k).dot(self.F_k.T) + self.Q

    def update(self, z_k):
        if z_k is None:
            return 1
        
        y_k = z_k - self.H_k.dot(self.s_k)
        S = self.H_k.dot(self.p_k).dot(self.H_k.T) + self.R
        d = self.mahalanobis_distance(y_k, S)
        if d > self.maha_thresh:
            return 1
        K  = self.p_k.dot(self.H_k.T).dot(np.linalg.inv(S))

        self.s_k = self.s_k + K.dot(y_k)
        self.p_k = (np.eye(self.n) - K.dot(self.H_k)).dot(self.p_k)
        return 0

    def mahalanobis_distance(self, y_k, S):
        invS = np.linalg.inv(S)
        d = y_k.dot(invS).dot(y_k)
        return d

    def step(self, measurement, confidence):
        self.predict()
        updated = False
        for meas, conf in zip(measurement, confidence):
            z_k = meas
            if conf < self.conf_thresh:
                continue
            if np.isnan(z_k).any():
                continue
            self.R = (1.1-conf)**2 * np.eye(int(self.n/2))
            res = self.update(z_k)
            if res:
                continue
            updated = True
        
        return self.s_k[:3].copy() if updated else np.array([np.nan, np.nan, np.nan])

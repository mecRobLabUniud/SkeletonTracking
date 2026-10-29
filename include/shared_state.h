/*
░█▀▀░█░█░█▀█░█▀▄░█▀▀░█▀▄░░░░░█▀▀░▀█▀░█▀█░▀█▀░█▀▀
░▀▀█░█▀█░█▀█░█▀▄░█▀▀░█░█░░░░░▀▀█░░█░░█▀█░░█░░█▀▀
░▀▀▀░▀░▀░▀░▀░▀░▀░▀▀▀░▀▀░░▀▀▀░▀▀▀░░▀░░▀░▀░░▀░░▀▀▀

shared_state.h - Cross-thread shared state: robot state, skeleton, stop flags, log ring buffer and log sample structure.
*/

#pragma once
#include <array>
#include <vector>
#include <mutex>
#include <atomic>
#include <cmath>

#include "min_distance_calculation.hpp"

//  Skeleton type 
// 15 joints or more, each a 3D point
using Skeleton = std::vector<std::array<double, 3>>;

//  Robot state 
// Written by control thread, read by collision checker thread
struct RobotState {
    std::array<double, 7> q;
    std::array<double, 7> q_p;
    std::array<double, 7> q_pp;
};

//  Shared variables 
inline std::mutex    robot_state_mutex;
inline RobotState    shared_robot_state;
inline std::atomic<double>  shared_rh_distance{1};

inline std::atomic<double> shared_stop_duration{0.4};

inline std::mutex    skeleton_mutex;
inline Skeleton shared_skeleton(21, std::array<double, 3>{NAN, NAN, NAN});

//  Thread control flags 
inline std::atomic<bool> stop_flag{false};
inline std::atomic<bool> trajectory_done{false};

// check if robot is stale
inline std::atomic<bool> skeleton_stale{false};

// logging utility
struct LogSample {
    double t;
    std::array<double, 7> q, qd, qdd;
    std::array<double, 3> p, pd;
    DistanceResult distance;
};

template <size_t N>
class SpcsRingBuffer {
public:
    bool push (const LogSample& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next = (head + 1) % N;
        if (next == tail_.load(std::memory_order_acquire)) return false; // full, so drop
        buf_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }
    
    bool pop (LogSample& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) return false; // empty
        item = buf_[tail];
        tail_.store((tail + 1) % N, std::memory_order_release);
        return true;
    }

private:
    std::array<LogSample, N> buf_;
    std::atomic<size_t> head_{0}, tail_{0};
};

inline SpcsRingBuffer<1024> log_queue;
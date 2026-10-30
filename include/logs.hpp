/*
░█░░░█▀█░█▀▀░█▀▀
░█░░░█░█░█░█░▀▀█
░▀▀▀░▀▀▀░▀▀▀░▀▀▀

logs.hpp - Header-only experiment metrics and MotionSuite-style XML logger used to record the runs.
*/

#pragma once

#include <iostream>
#include <fstream>

#include "shared_state.h"

static std::atomic<int> g_stop_count{0};        // R-STOPS
static double            g_idle_time = 0.0;      // seconds, robot velocity below threshold
static double            g_concurrent_ws_time = 0.0; // seconds, robot active & human in workspace
static double            g_t_log = 0.0;          // seconds, total accumulated control time (T-TIME)

// Debounce state for R-IDLE: tracks the length of the low-velocity streak
// currently in progress. Only written by the thread running the playback
// loop, so no synchronization needed (same reasoning as g_t_log/g_idle_time
// above).
static double            g_idle_streak_duration = 0.0;


static void resetMetrics() {
    g_stop_count.store(0);
    g_idle_time = 0.0;
    g_concurrent_ws_time = 0.0;
    g_t_log = 0.0;
    g_idle_streak_duration = 0.0;
}

std::array<std::string, 3> axis = {{"x", "y", "z"}};

void writeSample(std::ofstream& outfile_dist, 
                std::ofstream& outfile_q, 
                std::ofstream& outfile_qd, 
                std::ofstream& outfile_qdd,
                std::ofstream& outfile_p, 
                std::ofstream& outfile_pd, 
                const LogSample& s) {

    // joint positions
    outfile_q << "\t<keypoint time='" << s.t << "'>\n";
    for (int j = 0; j < 7; ++j)
        outfile_q << "\t\t<joint id='" << j << "'>" << s.q[j] << "</joint>\n";
    outfile_q << "\t</keypoint>\n";

    // joint velocities
    outfile_qd << "\t<keypoint time='" << s.t << "'>\n";
    for (int j = 0; j < 7; ++j)
        outfile_qd << "\t\t<joint id='" << j << "'>" << s.qd[j] << "</joint>\n";
    outfile_qd << "\t</keypoint>\n";

    // joint accelerations
    outfile_qdd << "\t<keypoint time='" << s.t << "'>\n";
    for (int j = 0; j < 7; ++j)
        outfile_qdd << "\t\t<joint id='" << j << "'>" << s.qdd[j] << "</joint>\n";
    outfile_qdd << "\t</keypoint>\n";

    // cartesian positions
    outfile_p << "\t<keypoint time='" << s.t << "'>\n";
    for (int j = 0; j < 3; ++j)
        outfile_p << "\t\t<point id='" << axis[j] << "'>" << s.p[j] << "</point>\n";
    outfile_p << "\t</keypoint>\n";

    // cartesian velocities
    outfile_pd << "\t<keypoint time='" << s.t << "'>\n";
    for (int j = 0; j < 3; ++j)
        outfile_pd << "\t\t<point id='" << axis[j] << "'>" << s.pd[j] << "</point>\n";
    outfile_pd << "\t</keypoint>\n";

    // Distance
    outfile_dist << "\t<keypoint time='" << s.t << "'>\n";
    outfile_dist << "\t\t<length>" << s.distance.length << "'</length>\n";
    outfile_dist << "\t\t<h_index>" << s.distance.ind_h << "'</h_index>\n";
    outfile_dist << "\t\t<c_h>\n";
    for (int j = 0; j < 3; ++j)
        outfile_dist << "\t\t\t<point id='" << axis[j] << "'>" << s.distance.c_h[j] << "</point>\n";
    outfile_dist << "\t\t</c_h>\n";
    outfile_dist << "\t\t<c_r>\n";
    for (int j = 0; j < 3; ++j)
        outfile_dist << "\t\t\t<point id='" << axis[j] << "'>" << s.distance.c_r[j] << "</point>\n";
    outfile_dist << "\t\t</c_r>\n\t</keypoint>\n";
}


void loggerThread(std::ofstream& outfile_dist, std::ofstream& outfile_q, std::ofstream& outfile_qd, std::ofstream& outfile_qdd,
                std::ofstream& outfile_p, std::ofstream& outfile_pd) {
    LogSample s;
    while (true) {
        bool drained_any = false;
        while (log_queue.pop(s)) {
            writeSample(outfile_dist, outfile_q, outfile_qd, outfile_qdd, outfile_p, outfile_pd, s);
            drained_any = true;
        }
        if (trajectory_done.load() && !drained_any) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    while (log_queue.pop(s)) writeSample(outfile_dist, outfile_q, outfile_qd, outfile_qdd, outfile_p, outfile_pd, s);
}
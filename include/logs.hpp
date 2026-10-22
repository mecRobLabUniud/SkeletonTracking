#pragma once

#include <iostream>
#include <fstream>

#include "shared_state.h"

void writeSample(std::ofstream& outfile_q,
                  std::ofstream& outfile_dq,
                  std::ofstream& outfile_tau,
                  std::ofstream& outfile_pose,
                  const LogSample& s) {

    // joint positions
    outfile_q << "\t<keypoint time='" << s.t << "'>\n";
    for (int j = 0; j < 7; ++j)
        outfile_q << "\t\t<point id='" << j << "'>" << s.q[j] << "</point>\n";
    outfile_q << "\t</keypoint>\n";

    // joint velocities
    outfile_dq << "\t<keypoint time='" << s.t << "'>\n";
    for (int j = 0; j < 7; ++j)
        outfile_dq << "\t\t<point id='" << j << "'>" << s.dq[j] << "</point>\n";
    outfile_dq << "\t</keypoint>\n";

    // commanded + measured torque
    outfile_tau << "\t<keypoint time='" << s.t << "'>\n";
    outfile_tau << "\t\t<tau_cmd>\n";
    for (int j = 0; j < 7; ++j)
        outfile_tau << "\t\t\t<point id='" << j << "'>" << s.tau_cmd[j] << "</point>\n";
    outfile_tau << "\t\t</tau_cmd>\n\t\t<tau_measured>\n";
    for (int j = 0; j < 7; ++j)
        outfile_tau << "\t\t\t<point id='" << j << "'>" << s.tau_meas[j] << "</point>\n";
    outfile_tau << "\t\t</tau_measured>\n\t</keypoint>\n";

    // 3D position
    outfile_pose << "\t<keypoint time='" << s.t << "'>\n";
    outfile_pose << "\t\t<ee_position>\n";
    outfile_pose << "\t\t\t<x>" << s.ee_pos[0] << "</x>\n";
    outfile_pose << "\t\t\t<y>" << s.ee_pos[1] << "</y>\n";
    outfile_pose << "\t\t\t<z>" << s.ee_pos[2] << "</z>\n";
    outfile_pose << "\t\t</ee_position>\n\t</keypoint>\n";
}


void loggerThread(std::ofstream& outfile_q, std::ofstream& outfile_dq, std::ofstream& outfile_tau, std::ofstream& outfile_pose) {
    LogSample s;
    while (true) {
        bool drained_any = false;
        while (log_queue.pop(s)) {
            writeSample(outfile_q, outfile_dq, outfile_tau, outfile_pose, s);
            drained_any = true;
        }
        if (trajectory_done.load() && !drained_any) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    while (log_queue.pop(s)) writeSample(outfile_q, outfile_dq, outfile_tau, outfile_pose, s); // final drain
}
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <array>
#include <stdexcept>
#include <cmath>
#include <sstream>
#include <string>
#include <iomanip>
#include <thread>

#include <franka/exception.h>
#include <franka/model.h>

/**
 * @example print_joint_poses.cpp
 * An example showing how to use the model library that prints the transformation
 * matrix of each joint with respect to the base frame.
 */

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}



void saveTrajectoryCSV(const std::string& path,
                       const std::vector<std::array<double, 7>>& traj) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot write: " + path);

    f << std::fixed << std::setprecision(9);
    f << "# q1,q2,q3,q4,q5,q6,q7\n";
    for (const auto& wp : traj) {
        for (int j = 0; j < 7; ++j) {
            f << wp[j];
            if (j < 6) f << ",";
        }
        f << "\n";
    }

    std::cout << "Saved " << traj.size() << " waypoints to " << path << "\n";
}



int main(int argc, char** argv) {
  // if (argc != 2) {
  //   std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  //   return -1;
  // }

  std::string trajectory_path = "src/trajectories/new_test/";

  try {
    franka::Robot robot("172.16.0.2");

    std::cout << "Recording.. Press 'q' and enter to stop the program" << std::endl;

    size_t count = 0;
    std::vector<std::array<double, 7>> q;
    std::vector<std::array<double, 7>> q_d;
    std::vector<std::array<double, 7>> dq;
    bool running = true;
    std::thread optimizer_thread([&]() {
      std::string tmp;
      std::cin >> tmp;
      running = false;
    });

    robot.read([&](const franka::RobotState& robot_state) {
      q.push_back(robot_state.q);
      q_d.push_back(robot_state.q_d);
      dq.push_back(robot_state.dq);
      return running;
    });

    saveTrajectoryCSV(trajectory_path + "q.csv",  q);
    saveTrajectoryCSV(trajectory_path + "q_d.csv",  q_d);
    saveTrajectoryCSV(trajectory_path + "dq.csv",  dq);

    std::cout << "Done." << std::endl;
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

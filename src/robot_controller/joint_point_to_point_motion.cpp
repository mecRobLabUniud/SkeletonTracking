// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.hpp"

/**
 * @example joint_point_to_point_motion.cpp
 * An example that moves the robot to a target position by commanding joint positions.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  // if (argc != 10) {
  //   std::cerr << "Usage: " << argv[0] << " <robot-hostname> "
  //             << "<joint0> <joint1> <joint2> <joint3> <joint4> <joint5> <joint6> "
  //             << "<speed-factor>" << std::endl
  //             << "joint0 to joint6 are joint angles in [rad]." << std::endl
  //             << "speed-factor must be between zero and one." << std::endl;
  //   return -1;
  // }


  std::array<double, 7> q_start = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
  std::array<double, 7> q_goal = {0.5, -0.7585, 0.0, -1.356, 0.0, 1.571, 0.785};
  double duration = 5.0; // seconds


  try {
    franka::Robot robot("172.16.0.2");
    setDefaultBehavior(robot);

    MotionGenerator motion_generator(0.5, q_start);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    

    std::array<double, 7> q_start;
    double t = 0.0;
    bool initialized = false;

    robot.control([&](const franka::RobotState& state, franka::Duration dt)
        -> franka::JointPositions {

        if (!initialized) {
            q_start = state.q;
            initialized = true;
        }

        t += dt.toSec();
        double s = std::min(t / duration, 1.0); // linear, 0→1

        // Smooth step (5th-order polynomial) — removes velocity discontinuities
        double s_smooth = 10*pow(s,3) - 15*pow(s,4) + 6*pow(s,5);

        std::array<double, 7> q_cmd;
        for (size_t i = 0; i < 7; i++) {
            q_cmd[i] = q_start[i] + s_smooth * (q_goal[i] - q_start[i]);
        }

        if (t >= duration) {
            return franka::MotionFinished(franka::JointPositions(q_goal));
        }
        return franka::JointPositions(q_cmd);
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

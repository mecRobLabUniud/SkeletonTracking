// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <fstream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.hpp"

/**
 * @example simple_control.cpp
 * An example showing how to generate a joint position motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

std::string trajectory_path = "src/trajectories/new_test/";






std::vector<std::array<double, 7>> loadTrajectoryCSV(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open: " + path);

    std::vector<std::array<double, 7>> traj;
    std::string line;

    while (std::getline(f, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        std::array<double, 7> wp;
        std::stringstream ss(line);
        std::string token;
        int j = 0;

        while (std::getline(ss, token, ',') && j < 7) {
            try {
                wp[j++] = std::stod(token);
            } catch (const std::exception&) {
                throw std::runtime_error("Bad value at line: " + line);
            }
        }

        if (j != 7)
            throw std::runtime_error("Expected 7 values, got "
                                     + std::to_string(j)
                                     + " at line: " + line);
        traj.push_back(wp);
    }

    std::cout << "Loaded " << traj.size() << " waypoints from " << path << "\n";
    return traj;
}








int main(int argc, char** argv) {
  // if (argc != 2) {
  //   std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  //   return -1;
  // }

  int columns = 7;
  bool stamp_results = false;

  try {
    franka::Robot robot("172.16.0.2");
    setDefaultBehavior(robot);

    // First move the robot to the starting waypoint
    // std::array<double, 7> q;
    // float t;

    // std::ifstream fileq(trajectory_path + "q_1kHz.csv");
    // if (fileq.is_open()) {
    //   for (int i=0; i<columns; i++) {
    //     fileq >> q[i];
    //   }  
    // }



    std::vector<std::array<double, 7>> q = loadTrajectoryCSV(trajectory_path + "q_1kHz.csv");





    std::cout << "Moving to initial joint configuration. " << q[0][0] << q[0][1] << q[0][2] << q[0][3] << std::endl;

    // std::ifstream filet(trajectory_path + "/t.txt");
    // if (filet.is_open()) {
    //   filet >> t;
    // }


    // std::ofstream fileqexp(trajectory_path + "/q_exp.txt");
    // std::ofstream fileqpexp(trajectory_path + "/q_p_exp.txt");
    // std::ofstream filetauexp(trajectory_path + "/tau_exp.txt");
    // std::ofstream filetexp(trajectory_path + "/t_exp.txt");

    MotionGenerator motion_generator(0.5, q[0]);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    double time = 0.0;
    double inst = 0.0; 
    int timestamp = 0; 
    std::array<double, 7> current_position;
    std::array<double, 7> current_velocity;
    std::array<double, 7> current_torque;
    

    std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)> 
    jointpositionscallback = [&](const franka::RobotState& robot_state,
                                                                franka::Duration period) -> franka::JointPositions {
      time += period.toSec();

      std::cout << timestamp << std::flush;

      
      std::array<double, 7> wp;
      for (int i=0; i<7; i++) wp[i] = q[timestamp][i];
      franka::JointPositions waypoint{wp};
      timestamp++;

      // current_position = robot_state.q;
      // current_velocity = robot_state.dq;
      // current_torque = robot_state.tau_J;

      /* if (stamp_results) {
        if (fileqexp.is_open()){
          for (int i=0; i<columns; i++){
            fileqexp << current_position[i] << '\t';
          }
          fileqexp << std::endl;
        }

        if (fileqpexp.is_open()){
          for (int i=0; i<columns; i++){
            fileqpexp << current_velocity[i] << '\t';
          }
          fileqpexp << std::endl;
        }

        if (filetauexp.is_open()){
          for (int i=0; i<columns; i++){
            filetauexp << current_torque[i] << '\t';
          }
          filetauexp << std::endl;
        }

        if (filetexp.is_open()){
          filetexp << time << std::endl;
        }
      } */

      if (timestamp >= q.size()) {
        std::cout << std::endl << "Finished motion, shutting down program" << std::endl;
        return franka::MotionFinished(waypoint);
      }
      return waypoint;
    };

    robot.control(jointpositionscallback);

    

    // fileq.close();
    // filetau.close();
    // filet.close();
    // fileqexp.close();
    // filetexp.close();

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

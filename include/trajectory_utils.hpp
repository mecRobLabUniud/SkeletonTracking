/*
░▀█▀░█▀▄░█▀█░▀▀█░█▀▀░█▀▀░▀█▀░█▀█░█▀▄░█░█░░░░░█░█░▀█▀░▀█▀░█░░░█▀▀
░░█░░█▀▄░█▀█░░░█░█▀▀░█░░░░█░░█░█░█▀▄░░█░░░░░░█░█░░█░░░█░░█░░░▀▀█
░░▀░░▀░▀░▀░▀░▀▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░░▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀▀▀░▀▀▀

trajectory_utils.hpp - Declaration of the quintic trajectory generator and of the point-to-point trajectory loader.
*/

#pragma once


#include <Eigen/Dense>
#include <optional>
#include <vector>
#include <string>

class RobotModel;

// ─────────────────────────────────────────────────────────────────────────────
// Quintic polynomial trajectory between two points (zero velocity and
// acceleration at both ends)
// ─────────────────────────────────────────────────────────────────────────────
struct Traj {
    Eigen::MatrixXd p, v, a;
};

// ─────────────────────────────────────────────────────────────────────────────
// Sample a quintic polynomial trajectory between x0 and x1
// ─────────────────────────────────────────────────────────────────────────────
Traj QuinticPolyTraj(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
                     double T, double dt, int n);

// ─────────────────────────────────────────────────────────────────────────────
// Time-indexed joint and Cartesian trajectory container
// ─────────────────────────────────────────────────────────────────────────────
struct Trajectory {
    std::vector<Eigen::VectorXd> q;
    std::vector<Eigen::VectorXd> qd;
    std::vector<Eigen::VectorXd> qdd;
    std::vector<Eigen::Vector3d> p;
    std::vector<Eigen::Vector3d> pd;
    std::vector<double> t;
};

// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point trajectory from the waypoints of a trajectory set
//
// The waypoints are read from <c_dir>/src/trajectories/test<n_traj>/ and can
// be given either in Cartesian coordinates (p_ref.csv, solved with the inverse
// kinematics) or in joint coordinates (q_ref.csv, used as they are). The
// cumulative time of each waypoint is read from t_ref.csv and gives the
// duration of each segment.
//
// When `t_restart` is given, only the waypoints reached before that time
// (timestamps strictly smaller than t_restart) are kept and the trajectory is
// rebuilt from them.
//
// When `q_restart` is given, it becomes the first waypoint of the rebuilt
// trajectory (replacing the first kept waypoint). The duration of the first
// segment is scaled by the ratio between the new and the recorded segment
// length, so that its playback speed is preserved while adapting the timing
// to the new path. `q_restart` must be a joint configuration.
//
// The two arguments are independent: either one alone is applied.
// ─────────────────────────────────────────────────────────────────────────────
Trajectory load_p2p_trajectory(RobotModel& robot, const std::string& c_dir,
                               int n_traj, double dt,
                               const std::optional<Eigen::VectorXd>& q_restart = std::nullopt,
                               const std::optional<double>& t_restart = std::nullopt);

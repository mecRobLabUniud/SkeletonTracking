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
// One time-stamped sample of a trajectory
// ─────────────────────────────────────────────────────────────────────────────
struct Waypoint {
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qdd;
    Eigen::Vector3d p;
    Eigen::Vector3d pd;
    double timestamp;
};

// ─────────────────────────────────────────────────────────────────────────────
// Time-indexed joint and Cartesian trajectory container
// ─────────────────────────────────────────────────────────────────────────────
struct Trajectory {
    std::vector<Waypoint> wp;
    std::string state = "pass";
};

// ─────────────────────────────────────────────────────────────────────────────
// Sample a quintic polynomial trajectory between x0 and x1
// ─────────────────────────────────────────────────────────────────────────────
Traj QuinticPolyTraj(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
                     double T, double dt, int n,
                     const Eigen::VectorXd& v0 = Eigen::VectorXd::Zero(7),
                     const Eigen::VectorXd& a0 = Eigen::VectorXd::Zero(7),
                     const Eigen::VectorXd& v1 = Eigen::VectorXd::Zero(7),
                     const Eigen::VectorXd& a1 = Eigen::VectorXd::Zero(7));

// ─────────────────────────────────────────────────────────────────────────────
// Append one quintic segment to the output trajectory
// ─────────────────────────────────────────────────────────────────────────────
void build_p2p_traj(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                    std::vector<double>* vnorm, Trajectory& out, const double t_end,
                    double& t, RobotModel& robot, double dt,
                    const Eigen::VectorXd& va = Eigen::VectorXd::Zero(7),
                    const Eigen::VectorXd& aa = Eigen::VectorXd::Zero(7),
                    const Eigen::VectorXd& vb = Eigen::VectorXd::Zero(7),
                    const Eigen::VectorXd& ab = Eigen::VectorXd::Zero(7));





double computeAddedTime(const Eigen::VectorXd& q_end, const Eigen::VectorXd& q_c);

// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point trajectory from the waypoints of a trajectory set
// ─────────────────────────────────────────────────────────────────────────────
Trajectory load_trajectory(RobotModel& robot, const std::string& c_dir,
                               int n_traj, double dt);

                               
// ─────────────────────────────────────────────────────────────────────────────
// Rebuild a point-to-point trajectory resuming from current motion state.
// Same structure as restart_trajectory, but the first rebuilt segment starts
// from the current velocity and acceleration instead of a steady state.
// ─────────────────────────────────────────────────────────────────────────────
void restart_trajectory(Trajectory& traj, RobotModel& robot,
                       const Eigen::VectorXd& q_resume,
                       const Eigen::VectorXd& qd_resume, const Eigen::VectorXd& qdd_resume,
                       const double& t_resume, double dt);

// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point trajectory with a trapezoidal velocity profile.
// Starts from a steady-state waypoint, accelerates to velocity_PFL, cruises at
// PFL, and finally decelerates to reach the following waypoint with zero
// velocity and acceleration.
// ─────────────────────────────────────────────────────────────────────────────
void limit_velocity(Trajectory& traj, RobotModel& robot,
                    const Eigen::VectorXd& q_start,
                    const Eigen::VectorXd& q_end,
                    const double& t_start, double dt,
                    double velocity_PFL);

// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point emergency stop trajectory. Returns the index of the
// last sample of the inserted stop (the end of the hold), so the caller knows
// when the stop has been fully played.
// ─────────────────────────────────────────────────────────────────────────────
size_t stop_trajectory(Trajectory& traj, RobotModel& robot,
                       const Eigen::VectorXd& q_stop,
                       const Eigen::VectorXd& qd_stop, const Eigen::VectorXd& qdd_stop,
                       const double& t_stop, const double& stop_duration, double dt);

// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point slow-down trajectory: instead of coming to a full
// stop, the robot is decelerated to a constant peak velocity equal to
// velocity_PFL and keeps following the previous trajectory. Returns the index
// of the last sample of the inserted slow-down segment.
// ─────────────────────────────────────────────────────────────────────────────
size_t slow_down_trajctory(Trajectory& traj, RobotModel& robot,
                           const Eigen::VectorXd& q_stop,
                           const Eigen::VectorXd& qd_stop, const Eigen::VectorXd& qdd_stop,
                           const double& t_stop, const double& stop_duration, double dt,
                           double velocity_PFL);
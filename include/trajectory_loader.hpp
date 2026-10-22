/*
░▀█▀░█▀▄░█▀█░▀▀█░█▀▀░█▀▀░▀█▀░█▀█░█▀▄░█░█░░░░░█░░░█▀█░█▀█░█▀▄░█▀▀░█▀▄
░░█░░█▀▄░█▀█░░░█░█▀▀░█░░░░█░░█░█░█▀▄░░█░░░░░░█░░░█░█░█▀█░█░█░█▀▀░█▀▄
░░▀░░▀░▀░▀░▀░▀▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░░▀░░▀▀▀░▀▀▀░▀▀▀░▀░▀░▀▀░░▀▀▀░▀░▀

trajectory_loader.hpp - Declaration of the CSV trajectory loader and of the TrajectoryData structure.
*/

#pragma once

#include <Eigen/Dense>
#include <filesystem>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// A robot trajectory as stored on disk: the waypoints (either Cartesian
// coordinates or joint configurations) and the cumulative time at which each
// of them is reached.
//
// The trajectories live in src/trajectories:
//   traiettoria.txt / t.txt        the plain format used by the demo
//   testN/p_ref.csv + t_ref.csv   Cartesian waypoints of the set testN
//   testN/q_ref.csv + t_ref.csv   joint waypoints of the set testN
// ─────────────────────────────────────────────────────────────────────────────
struct TrajectoryData {
    std::vector<Eigen::Vector3d> p;  // Cartesian waypoints (empty if the file holds joint values)
    std::vector<Eigen::VectorXd> q;  // joint waypoints (empty if the file holds Cartesian coordinates)
    std::vector<double> t;           // cumulative time of each waypoint

    // True when the file holds joint configurations instead of Cartesian points
    bool isJointSpace() const { return !q.empty(); }

    // The cumulative times tell when each waypoint is reached. When the first
    // one is reached at t = 0 the robot starts there and the playback begins
    // with the second waypoint; otherwise the robot starts from the home
    // configuration and the first segment reaches the first waypoint at t[0].
    bool startsAtFirstWaypoint() const { return !t.empty() && t.front() == 0.0; }
};

// Read a trajectory from a directory: the waypoints from <points_file> and the
// cumulative times from <times_file>. The waypoint file holds either three
// values per row (Cartesian x,y,z) or seven (joint configuration); commas and
// blanks are both accepted as separators.
TrajectoryData loadTrajectory(const std::filesystem::path& dir,
                              const std::string& points_file,
                              const std::string& times_file);

// Read a trajectory set from a directory: the waypoints from p_ref.csv
// (Cartesian) or q_ref.csv (joint) and the cumulative times from t_ref.csv.
TrajectoryData loadTrajectorySet(const std::filesystem::path& dir);

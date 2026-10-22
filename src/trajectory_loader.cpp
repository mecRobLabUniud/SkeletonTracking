/*
░▀█▀░█▀▄░█▀█░▀▀█░█▀▀░█▀▀░▀█▀░█▀█░█▀▄░█░█░░░░░█░░░█▀█░█▀█░█▀▄░█▀▀░█▀▄
░░█░░█▀▄░█▀█░░░█░█▀▀░█░░░░█░░█░█░█▀▄░░█░░░░░░█░░░█░█░█▀█░█░█░█▀▀░█▀▄
░░▀░░▀░▀░▀░▀░▀▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░░▀░░▀▀▀░▀▀▀░▀▀▀░▀░▀░▀▀░░▀▀▀░▀░▀

trajectory_loader.cpp - CSV loader for recorded reference trajectories (Cartesian waypoints or joint configurations plus their timing).
*/

#include "trajectory_loader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {

// ─────────────────────────────────────────────────────────────────────────────
// Read a CSV file into rows of numbers.
//
// Blank lines and comment lines (starting with '#', as used by the header of
// the reference files) are skipped. Both commas and blanks are accepted as
// column separators, and a trailing '\r' (CRLF files) is tolerated.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<std::vector<double>> read_csv_rows(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open " + path.string());

    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();

        // Skip blank lines and comments
        const size_t first = line.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        if (line[first] == '#') continue;

        // Split on commas; a cell may itself hold several blanks-separated
        // values, so each cell is further split on whitespace.
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            std::stringstream cs(cell);
            std::string token;
            while (cs >> token) {
                try {
                    row.push_back(std::stod(token));
                } catch (const std::exception&) {
                    throw std::runtime_error(path.string() + ": cannot parse '" + token + "'");
                }
            }
        }
        if (!row.empty()) rows.push_back(std::move(row));
    }
    return rows;
}

}  // namespace


// ─────────────────────────────────────────────────────────────────────────────
// Read a trajectory from a directory: the waypoints from <points_file> and the
// cumulative times from <times_file>.
//
// The waypoint file holds either three values per row (Cartesian x, y, z) or
// seven (joint configuration); the format is detected from the row width.
// ─────────────────────────────────────────────────────────────────────────────
TrajectoryData loadTrajectory(const std::filesystem::path& dir,
                              const std::string& points_file,
                              const std::string& times_file) {
    TrajectoryData traj;

    // ── Waypoints ────────────────────────────────────────────────────────────
    const std::filesystem::path points_path = dir / points_file;
    const auto rows = read_csv_rows(points_path);
    if (rows.size() < 2) {
        throw std::runtime_error(points_path.string() + ": need at least 2 waypoints");
    }

    const size_t width = rows.front().size();
    for (const auto& r : rows) {
        if (r.size() != width) {
            throw std::runtime_error(points_path.string() + ": inconsistent number of columns");
        }
    }

    if (width == 3) {
        for (const auto& r : rows) traj.p.emplace_back(r[0], r[1], r[2]);
    } else if (width == 7) {
        for (const auto& r : rows) {
            Eigen::VectorXd q(7);
            for (int j = 0; j < 7; ++j) q(j) = r[j];
            traj.q.push_back(std::move(q));
        }
    } else {
        throw std::runtime_error(points_path.string() +
                                 ": each row must have exactly 3 (Cartesian) or 7 (joint) values");
    }

    // ── Cumulative times ─────────────────────────────────────────────────────
    const std::filesystem::path times_path = dir / times_file;
    for (const auto& r : read_csv_rows(times_path)) {
        if (r.empty()) continue;
        traj.t.push_back(r[0]);
    }

    if (traj.t.size() != rows.size()) {
        throw std::runtime_error(times_path.string() + ": expected " +
                                 std::to_string(rows.size()) + " time values, got " +
                                 std::to_string(traj.t.size()));
    }

    // The times must be non-decreasing, otherwise the segments would have a
    // negative duration
    for (size_t i = 1; i < traj.t.size(); ++i) {
        if (traj.t[i] < traj.t[i - 1]) {
            throw std::runtime_error(times_path.string() + ": times must be non-decreasing");
        }
    }

    return traj;
}


// ─────────────────────────────────────────────────────────────────────────────
// Read a trajectory set from a directory: the waypoints from p_ref.csv
// (Cartesian) or q_ref.csv (joint) and the cumulative times from t_ref.csv.
// ─────────────────────────────────────────────────────────────────────────────
TrajectoryData loadTrajectorySet(const std::filesystem::path& dir) {
    std::error_code ec;
    const bool has_cartesian = std::filesystem::exists(dir / "p_ref.csv", ec);
    const bool has_joint     = std::filesystem::exists(dir / "q_ref.csv", ec);

    if (has_cartesian && has_joint) {
        throw std::runtime_error(dir.string() +
                                 ": both p_ref.csv and q_ref.csv present, the waypoint format is ambiguous");
    }
    if (!has_cartesian && !has_joint) {
        throw std::runtime_error(dir.string() + ": no p_ref.csv or q_ref.csv found");
    }

    return loadTrajectory(dir, has_cartesian ? "p_ref.csv" : "q_ref.csv", "t_ref.csv");
}

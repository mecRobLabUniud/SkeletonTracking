#pragma once
#include <Eigen/Core>
#include <array>
#include <limits>
#include <vector>
#include <nlohmann/json.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// Convert a JSON value to double, mapping null to NaN
// ─────────────────────────────────────────────────────────────────────────────
inline double json_to_double(const nlohmann::json& v) {
    if (v.is_null()) return std::numeric_limits<double>::quiet_NaN();
    return v.get<double>();
}


// ─────────────────────────────────────────────────────────────────────────────
// Convert a JSON array of 3D points into a vector of Eigen 3D vectors
// ─────────────────────────────────────────────────────────────────────────────
inline std::vector<Eigen::Vector3d> json_to_keypoints(const nlohmann::json& arr) {
    std::vector<Eigen::Vector3d> out;
    out.reserve(arr.size());
    for (const auto& p : arr) {
        out.push_back({ json_to_double(p[0]), json_to_double(p[1]), json_to_double(p[2]) });
    }
    return out;
}

/*
waypoint_zmq.hpp - Serialization of a trajectory Waypoint for the ZeroMQ link
between main_SSM.cpp and the real-robot bridge.

The message is published on topic WAYPOINT_15 (port 6015) as a single JSON
object with the fields q, qd, qdd, p, pd and t.  This is the same
topic-tagged/semicolon-separated framing used by data_transmitter.hpp.
*/

#pragma once

#include <nlohmann/json.hpp>
#include <Eigen/Dense>

#include <stdexcept>
#include <string>
#include <vector>

#include "trajectory_utils.hpp"

namespace waypoint_zmq {

// Device id used by the DataTransmitter (port = kBasePort + kDeviceId).
inline constexpr int kDeviceId = 15;
inline constexpr int kBasePort = 6000;
inline constexpr const char* kTopic = "WAYPOINT";

// ─────────────────────────────────────────────────────────────────────────────
// Waypoint -> JSON
// ─────────────────────────────────────────────────────────────────────────────
inline nlohmann::json encode(const Waypoint& wp) {
    const auto to_array = [](const Eigen::VectorXd& v) {
        return std::vector<double>(v.data(), v.data() + v.size());
    };

    nlohmann::json j;
    j["q"]   = to_array(wp.q);
    j["qd"]  = to_array(wp.qd);
    j["qdd"] = to_array(wp.qdd);
    j["p"]   = std::vector<double>(wp.p.data(), wp.p.data() + wp.p.size());
    j["pd"]  = std::vector<double>(wp.pd.data(), wp.pd.data() + wp.pd.size());
    j["t"]   = wp.timestamp;
    return j;
}

// ─────────────────────────────────────────────────────────────────────────────
// JSON -> Waypoint (validates the vector lengths)
// ─────────────────────────────────────────────────────────────────────────────
inline Eigen::VectorXd decode_vector(const nlohmann::json& value, Eigen::Index expected,
                                     const char* name) {
    const std::vector<double> values = value.get<std::vector<double>>();
    if (expected > 0 && static_cast<Eigen::Index>(values.size()) != expected) {
        throw std::runtime_error(
            std::string("waypoint_zmq: '") + name + "' has " +
            std::to_string(values.size()) + " values, expected " +
            std::to_string(expected));
    }
    return Eigen::Map<const Eigen::VectorXd>(values.data(),
                                             static_cast<Eigen::Index>(values.size()));
}

inline Waypoint decode(const nlohmann::json& j) {
    Waypoint wp;
    wp.q   = decode_vector(j.at("q"), 7, "q");
    wp.qd  = decode_vector(j.at("qd"), 7, "qd");
    wp.qdd = decode_vector(j.at("qdd"), 7, "qdd");

    const Eigen::VectorXd p  = decode_vector(j.at("p"), 3, "p");
    const Eigen::VectorXd pd = decode_vector(j.at("pd"), 3, "pd");
    wp.p  = Eigen::Vector3d(p(0), p(1), p(2));
    wp.pd = Eigen::Vector3d(pd(0), pd(1), pd(2));

    wp.timestamp = j.at("t").get<double>();
    return wp;
}

}  // namespace waypoint_zmq

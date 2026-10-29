/*
░█▀▀░█░█░█▀▀░█░░░█▀▀░▀█▀░█▀█░█▀█░░░░░█▀▄░█▀▀░█▀█░█▀▄░█▀▀░█▀▄
░▀▀█░█▀▄░█▀▀░█░░░█▀▀░░█░░█░█░█░█░░░░░█▀▄░█▀▀░█▀█░█░█░█▀▀░█▀▄
░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░▀▀▀░▀░▀░▀▀▀░▀░▀░▀▀░░▀▀▀░▀░▀

skeleton_reader.cpp - Background thread that subscribes to the merged multi-camera skeleton and publishes it into the shared state.
*/

#include "skeleton_reader.h"
#include "data_transmitter.hpp"
#include "utils.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>


//  receiveSkeletonMerged helpers 

// Builds a Skeleton of `n` points, every coordinate set to NaN. Used to
// populate shared_skeleton when no valid frame is available (startup, or
// after the stream goes stale), so downstream consumers that gate on
// !skeleton.empty() (e.g. stop-duration/radii logging) still run instead of
// silently skipping — the NaNs are the "no data" signal for their existing
// NaN-aware distance calculations to catch.
static Skeleton makeNaNSkeleton(size_t n) {
    return Skeleton(n, Eigen::Vector3d::Constant(NAN));
}

// Converts one JSON point of the merged skeleton message (a 3-element array,
// as produced by json.dumps() in utils/data_transmitter.py._send_skeleton_data)
// into a Skeleton entry. Python's json module emits the bare, non-standard
// token "NaN" for missing values; DataTransmitter::receive_data() sanitizes
// those to JSON null before parsing, and json_to_double (utils.hpp) maps null
// back to a quiet NaN, which is the "no data" signal downstream code expects.
// Returns false on structural malformation (not an array / too short).
static bool jsonPointToVector(const nlohmann::json& p, Eigen::Vector3d& out) {
    if (!p.is_array() || p.size() < 3) return false;
    out = Eigen::Vector3d(json_to_double(p[0]),
                          json_to_double(p[1]),
                          json_to_double(p[2]));
    return true;
}

// Live mode: subscribes (via DataTransmitter, the C++ mirror of
// utils.data_transmitter.DataTransmitter) to the merged multi-camera skeleton
// stream published by data_merging.py — topic "MERGED", device_id 10. The
// receive is polled with POLL_TIMEOUT_MS so trajectory_done and skeleton
// staleness are re-checked even when no frames arrive.
// Writes to shared_skeleton, and sets skeleton_stale if no valid frame has
// arrived within STALE_TIMEOUT_S.
void receiveSkeletonMerged(const std::string& host, int device_id, const std::string& topic,
                            const std::string& log_path) {
    (void)host;  // DataTransmitter connects to localhost, mirroring the Python side

    // Fail-safe: if no valid frame has arrived within this long, flag the
    // skeleton data as stale so the collision checker forces a stop rather
    // than continuing to trust a frozen (or never-received) skeleton.
    constexpr double STALE_TIMEOUT_S = 0.1;
    constexpr int    POLL_TIMEOUT_MS = 100;  // re-check trajectory_done / staleness this often
    constexpr size_t MERGED_SKELETON_JOINTS = 21;  // camera/merged joint count (vs. 15 for XML playback)

    // logging, opened once at thread start
    std::ofstream log_file;
    if (!log_path.empty()) {
        log_file.open(log_path, std::ios::out);
        if (!log_file.is_open()) {
            std::cerr << "skeleton_reader: WARNING could not open log file '"
                       << log_path << "' — continuing without logging\n";
        } else {
            std::cout << "skeleton_reader: logging received frames to '" << log_path << "'\n";
        }
    }

    // Seed with a NaN-filled skeleton immediately, so consumers gating on
    // !skeleton.empty() see valid (if NaN) data from t=0 rather than an
    // empty vector during the window before the first staleness check
    // below has a chance to run.
    {
        std::lock_guard<std::mutex> lock(skeleton_mutex);
        shared_skeleton = makeNaNSkeleton(MERGED_SKELETON_JOINTS);
    }

    std::cout << "skeleton_reader: subscribing to '" << topic << "_" << device_id
               << "' published by data_merging.py\n";

    // The merged skeleton is published by data_merging.py on topic "MERGED",
    // device_id 10 -> port 6000 + 10 = 6010 (the Python side uses the same
    // "port + device_id" convention). No shared-memory image buffer is
    // needed here: this thread only consumes the JSON payload.
    DataTransmitter dtr(DataTransmitter::Mode::Receiver, device_id, topic,
                        6000, /*use_shm=*/false);
    dtr.set_receive_timeout(POLL_TIMEOUT_MS);  // conflate is set internally: we only ever want the latest frame

    auto last_valid_frame = std::chrono::steady_clock::now();
    bool warned_stale = false;

    while (!trajectory_done.load()) {
        std::vector<nlohmann::json> arrays = dtr.receive_data();

        if (!arrays.empty()) {
            // First array is the merged skeleton positions ("[[x,y,z], ...]");
            // the second (merged confidence) is all-ones in data_merging.py and
            // carries no per-joint validity information, so it is ignored —
            // NaN in the positions is the only signal acted on downstream.
            Skeleton parsed;
            for (const auto& p : arrays[0]) {
                Eigen::Vector3d pt;
                if (!jsonPointToVector(p, pt)) {
                    parsed.clear();  // structural malformation: reject the frame
                    break;
                }
                parsed.push_back(pt);
            }

            if (!parsed.empty()) {
                if (log_file.is_open()) {
                    log_file << parsed.size() << "\n";
                    for (size_t i = 0; i < parsed.size(); ++i) {
                        const auto& pt = parsed[i];
                        log_file << "  [" << i << "] -> ";
                        if (pt.hasNaN()) {
                            log_file << "NaN, NaN, NaN\n";
                        } else {
                            log_file << "x: " << pt.x() << ", y: " << pt.y() << ", z: " << pt.z() << "\n";
                        }
                    }
                }
                {
                    std::lock_guard<std::mutex> lock(skeleton_mutex);
                    shared_skeleton = std::move(parsed);
                }

                last_valid_frame = std::chrono::steady_clock::now();
                skeleton_stale.store(false);

                if (warned_stale) {
                    std::cerr << "skeleton_reader: recovered, receiving fresh frames again\n";
                    warned_stale = false;
                }
            } else {
                std::cerr << "skeleton_reader: discarding malformed skeleton message\n";
            }
        }

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_valid_frame).count();

        // last_valid_frame starts at thread-launch time, so this timeout
        // applies equally to "never received a frame at all" and to
        // "was receiving, then stopped" — both are the same failure mode
        // from the collision checker's point of view.
        if (elapsed > STALE_TIMEOUT_S) {
            skeleton_stale.store(true);
            if (!warned_stale) {
                std::cerr << "skeleton_reader: WARNING no valid skeleton frame in "
                           << elapsed << "s (> " << STALE_TIMEOUT_S
                           << "s) — flagging stale\n";
                warned_stale = true;

                std::lock_guard<std::mutex> lock(skeleton_mutex);
                shared_skeleton = makeNaNSkeleton(MERGED_SKELETON_JOINTS);
            }
        }
    }

    dtr.shutdown();

    if (log_file.is_open()) {
        log_file.close();
        std::cout << "skeleton_reader: closed log file '" << log_path << "'\n";
    }
}
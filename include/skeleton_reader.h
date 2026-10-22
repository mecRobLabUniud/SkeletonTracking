/*
░█▀▀░█░█░█▀▀░█░░░█▀▀░▀█▀░█▀█░█▀█░░░░░█▀▄░█▀▀░█▀█░█▀▄░█▀▀░█▀▄
░▀▀█░█▀▄░█▀▀░█░░░█▀▀░░█░░█░█░█░█░░░░░█▀▄░█▀▀░█▀█░█░█░█▀▀░█▀▄
░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░▀▀▀░▀░▀░▀▀▀░▀░▀░▀▀░░▀▀▀░▀░▀

skeleton_reader.h - Declaration of the background subscriber that keeps the shared skeleton up to date.
*/

#pragma once
#include <string>
#include "shared_state.h"


// Live mode: subscribes over ZeroMQ (via DataTransmitter, the C++ mirror of
// utils.data_transmitter.DataTransmitter) to the merged multi-camera skeleton
// stream published by data_merging.py (topic "MERGED", device_id 10 ->
// tcp port 6000 + 10 = 6010). Assumes data_merging.py and this process run
// on the same host (per DataTransmitter's hardcoded "tcp://localhost:...").
// Runs in its own thread, writes to shared_skeleton, and sets
// skeleton_stale if no valid frame has arrived within STALE_TIMEOUT_S.
void receiveSkeletonMerged(const std::string& host = "localhost",
                            int device_id = 10,
                            const std::string& topic = "MERGED",
                            const std::string& log_path = "");

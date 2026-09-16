/*
░█▀▄░█░█░█░░░█▀█░░░█▀▀░█░█░█▀█░█░░░█░█░█▀█░▀█▀░▀█▀░█▀█░█▀█
░█▀▄░█░█░█░░░█▀█░░░█▀▀░▀▄▀░█▀█░█░░░█░█░█▀█░░█░░░█░░█░█░█░█
░▀░▀░▀▀▀░▀▀▀░▀░▀░░░▀▀▀░░▀░░▀░▀░▀▀▀░▀▀▀░▀░▀░░▀░░▀▀▀░▀▀▀░▀░▀
*/

#include <string>
#include <array>
#include <vector>

#include "rula_score_computation.hpp"
#include "data_transmitter.hpp"
#include "utils.hpp"


// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    AdjustmentFlags flags;
    flags.isRepeated    = false;
    flags.forceScoreA   = 0;
    flags.forceScoreB   = 0;

    DataTransmitter dtr = DataTransmitter(DataTransmitter::Mode::Receiver, 10, "MERGED");
    DataTransmitter dts = DataTransmitter(DataTransmitter::Mode::Sender, 11, "RULA");
    while (true) {
        auto skeleton = json_to_keypoints(dtr.receive_data()[0]);

        RULAResult result_R = computeRULA(skeleton, flags, 'R', false);
        RULAResult result_L = computeRULA(skeleton, flags, 'L', false);

        std::array<int, 2> rula_score = {result_R.grandScore, result_L.grandScore};
        dts.send_rula_score(rula_score);
    }
}
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
// Receive merged skeletons, evaluate the RULA score for both sides and
// publish the two scores
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    // ── Adjustment flags ────────────────────────────────────────────────────
    AdjustmentFlags flags;

    // ── Data transmitters ───────────────────────────────────────────────────
    DataTransmitter dtr = DataTransmitter(DataTransmitter::Mode::Receiver, 10, "MERGED");
    DataTransmitter dts = DataTransmitter(DataTransmitter::Mode::Sender, 11, "RULA");

    // ── Evaluation loop ─────────────────────────────────────────────────────
    while (true) {
        auto skeleton = json_to_keypoints(dtr.receive_data()[0]);

        RULAResult result_R = computeRULA(skeleton, flags, 'R', false);
        RULAResult result_L = computeRULA(skeleton, flags, 'L', false);

        std::array<int, 2> rula_score = {result_R.grandScore, result_L.grandScore};
        dts.send_rula_score(rula_score);
    }
}

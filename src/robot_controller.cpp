#include <franka/robot.h>
#include <franka/exception.h>
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#include "utils.hpp"
#include "data_transmitter.hpp"
#include "trajectory_utils.hpp"

// struct Sample { double t; std::array<double,7> q, dq; };

/*std::array<double,7> hermite(const Sample& a, const Sample& b, double t) {
  double h = b.t - a.t;
    double s = (t - a.t) / h;
    double h00 =  2*s*s*s - 3*s*s + 1;
    double h10 =      s*s*s - 2*s*s + s;
    double h01 = -2*s*s*s + 3*s*s;
    double h11 =      s*s*s -   s*s;
    std::array<double,7> q;
    for (int i = 0; i < 7; ++i)
        q[i] = h00*a.q[i] + h10*h*a.dq[i] + h01*b.q[i] + h11*h*b.dq[i];
    return q;
}


std::array<double,7> quintic(const WP& a, const WP& b, double t) {
    double T = b.t - a.t, s = (t - a.t) / T;
    double s2=s*s, s3=s2*s, s4=s3*s, s5=s4*s;
    double h0 = 1 - 10*s3 + 15*s4 - 6*s5;
    double h1 = s - 6*s3 + 8*s4 - 3*s5;
    double h2 = 0.5*s2 - 1.5*s3 + 1.5*s4 - 0.5*s5;
    double h3 = 10*s3 - 15*s4 + 6*s5;
    double h4 = -4*s3 + 7*s4 - 3*s5;
    double h5 = 0.5*s3 - s4 + 0.5*s5;
    std::array<double,7> q;
    for (int i = 0; i < 7; ++i)
        q[i] = h0*a.q[i] + h1*T*a.dq[i] + h2*T*T*a.ddq[i]
            + h3*b.q[i] + h4*T*b.dq[i] + h5*T*T*b.ddq[i];
    return q;
}*/

static Waypoint receiveRobotStates(DataTransmitter& tx) {
        const std::vector<nlohmann::json> data = tx.receive_data();
        std::cout << "data[2]: " << data[2] << std::endl;
        std::cout << "data[3]: " << data[3] << std::endl;

        if (data.empty()) return {};
        Waypoint wp;
        wp.q = json_to_robotstate(data[2]);
        wp.qd = json_to_robotstate(data[3]);
        return wp;
    }


int main() {
    franka::Robot robot("172.16.0.2");
    robot.setCollisionBehavior(
        {{20,20,18,18,16,14,12}}, {{20,20,18,18,16,14,12}},
        {{20,20,18,18,16,14,12}}, {{20,20,18,18,16,14,12}},
        {{20,20,20,25,25,25}}, {{20,20,20,25,25,25}},
        {{20,20,20,25,25,25}}, {{20,20,20,25,25,25}});

    // ── Data transmitters ───────────────────────────────────────────────────
    DataTransmitter transmitter = DataTransmitter(DataTransmitter::Mode::Receiver, 12, "ROBOT");

    Waypoint waypoint;

    while (true) {
        waypoint = receiveRobotStates(transmitter);
        std::cout << "q: " << waypoint.q.transpose() << " - qd: "  << waypoint.qd.transpose() << std::endl;
    }

    /*double time = 0.0;
    size_t k = 0;
    try {
        robot.control([&](const franka::RobotState& state,
                        franka::Duration period) -> franka::JointPositions {
        // time += period.toSec();
        // if (time == period.toSec()) { // primo ciclo: verifica che state.q ≈ traj[0].q }
    // 
        // while (k + 2 < traj.size() && time > traj[k+1].t) ++k;
        // double tt = std::min(time, traj.back().t);
        // franka::JointPositions out(hermite(traj[k], traj[k+1], tt));
        // out.motion_finished = (time >= traj.back().t);
        // return out;

        struct Segment { };
        std::array<Segment, 2> buf;
        std::atomic<int> active{0};

        // thread 60 Hz (non RT): scrive nel buffer inattivo, poi swap
        int idle = 1 - active.load(std::memory_order_acquire);
        buf[idle] = build_blended_segment(// ultimo q_d, dq_d, nuovi waypoint);
        active.store(idle, std::memory_order_release);

        // callback 1 kHz: legge solo buf[active], nessun lock né allocazione

        // loop RT (callback di robot.control)
            time += period.toSec();
            while (time >= w1.t) {
            WP next;
            if (queue.pop(next)) { w0 = w1; w1 = next; }
            return franka::JointPositions(quintic(w0, w1, std::min(time, w1.t)));
        });
    } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
    }*/
}
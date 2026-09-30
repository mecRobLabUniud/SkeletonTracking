// C++ port of "Thesis - Different Methods" (PFL & SSM & escape branch only).
// Uses your RobotModel (Pinocchio) and SSMPFL() as given.
//
// Build (adapt to your CMake): link against Eigen3 + pinocchio.
//
// Differences vs MATLAB worth knowing:
//   * Jacobian ordering: Pinocchio is [linear; angular], MATLAB's
//     geometricJacobian is [angular; linear]. So "vf(4:6,:)" -> head<3>() here.
//   * COLLcheck_franka was not provided -> CollisionFree() below is a
//     placeholder (link-origin segments vs. a point with clearance). Replace
//     with your real check if you have one.
//   * Qpj / Qpt are new arguments of your C++ SSMPFL that the MATLAB call did
//     not have -> set them to whatever your MATLAB SSMPFL_franka used.
//   * Plots are replaced by a CSV dump (paths.csv) you can plot with anything.

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdio>

#include "robot_model.hpp"  // RobotModel
#include "SSMPFL.hpp"       // SSMPFLResult, SSMPFL
#include "data_transmitter.hpp"


// Sends one 3D dataset to an open gnuplot pipe, terminated with "e".
template <typename GetPoint>
static void SendXYZ(FILE* gp, int n, GetPoint get) {
  for (int k = 0; k < n; ++k) {
    const Eigen::Vector3d p = get(k);
    std::fprintf(gp, "%f %f %f\n", p.x(), p.y(), p.z());
  }
  std::fprintf(gp, "e\n");
}

void PlotPaths(const std::vector<Eigen::Vector3d>& p_real, const Eigen::MatrixXd& p_int,
               const Eigen::MatrixXd& pf, const Eigen::MatrixXd& ps) {
  FILE* gp = popen("gnuplot -persist", "w");
  if (!gp) { std::cerr << "gnuplot not found\n"; return; }

  std::fprintf(gp,
        "set title 'Path of both manipulator and operator'\n"
        "set xlabel 'x [m]'; set ylabel 'y [m]'; set zlabel 'z [m]'\n"
        "set view equal xyz\n"
        "set view 70, 125\n"          // <-- elevation, azimuth
        "set grid\n"
        "splot '-' with lines lc rgb '#0072BD' title 'robot (real)', \\\n"
        "      '-' with lines lc rgb 'red'     title 'human', \\\n"
        "      '-' with lines lc rgb 'green'   title 'nominal forward', \\\n"
        "      '-' with lines lc rgb 'green'   title 'nominal backward'\n");

  SendXYZ(gp, static_cast<int>(p_real.size()), [&](int k) { return p_real[k]; });
  SendXYZ(gp, static_cast<int>(p_int.cols()),  [&](int k) { return Eigen::Vector3d(p_int.col(k)); });
  SendXYZ(gp, static_cast<int>(pf.cols()),     [&](int k) { return Eigen::Vector3d(pf.col(k)); });
  SendXYZ(gp, static_cast<int>(ps.cols()),     [&](int k) { return Eigen::Vector3d(ps.col(k)); });

  pclose(gp);
}



// ----------------------------------------------------------------------------
// Quintic polynomial trajectory between two points (zero vel/acc at the ends),
// equivalent to quinticpolytraj(...) with 2 waypoints.
// ----------------------------------------------------------------------------
struct Traj {
  Eigen::MatrixXd p, v, a;  // dim x n
};

Traj QuinticPolyTraj(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1, double T,
                     double dt, int n) {
  const int d = static_cast<int>(x0.size());
  Traj tr{Eigen::MatrixXd(d, n), Eigen::MatrixXd(d, n), Eigen::MatrixXd(d, n)};
  const Eigen::VectorXd dx = x1 - x0;
  for (int k = 0; k < n; ++k) {
    const double t = std::min(k * dt, T);
    const double s = t / T;
    const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    const double h = 10 * s3 - 15 * s4 + 6 * s5;
    const double dh = (30 * s2 - 60 * s3 + 30 * s4) / T;
    const double ddh = (60 * s - 180 * s2 + 120 * s3) / (T * T);
    tr.p.col(k) = x0 + dx * h;
    tr.v.col(k) = dx * dh;
    tr.a.col(k) = dx * ddh;
  }
  return tr;
}

// ----------------------------------------------------------------------------
// Placeholder for COLLcheck_franka. Returns TRUE when there is NO collision
// (matches the MATLAB usage: `if not(COLLcheck_franka(...))` -> collision).
// ----------------------------------------------------------------------------
static double PointSegmentDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& a,
                                   const Eigen::Vector3d& b) {
  const Eigen::Vector3d ab = b - a;
  const double len2 = ab.squaredNorm();
  double t = len2 > 1e-12 ? (p - a).dot(ab) / len2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  return (p - (a + t * ab)).norm();
}

bool CollisionFree(const RobotModel& robot, const Eigen::VectorXd& q,
                   const Eigen::Vector3d& p, double clearance) {
  static const char* kLinks[] = {"panda_link1", "panda_link2", "panda_link3",
                                 "panda_link4", "panda_link5", "panda_link6",
                                 "panda_link7", "panda_link8"};
  robot.ComputeFK(q);
  Eigen::Vector3d prev = Eigen::Vector3d::Zero();  // base origin
  for (const char* name : kLinks) {
    const Eigen::Vector3d cur = robot.GetJointPose(name).translation();
    if (PointSegmentDistance(p, prev, cur) < clearance) return false;
    prev = cur;
  }
  return true;
}

static void WriteCsv(const std::string& path, const std::vector<Eigen::Vector3d>& v) {
  std::ofstream f(path);
  f << "x,y,z\n";
  for (const auto& p : v) f << p.x() << "," << p.y() << "," << p.z() << "\n";
}
static void WriteCsv(const std::string& path, const Eigen::MatrixXd& m) {
  std::ofstream f(path);
  f << "x,y,z\n";
  for (int k = 0; k < m.cols(); ++k)
    f << m(0, k) << "," << m(1, k) << "," << m(2, k) << "\n";
}

int main(int argc, char** argv) {
  const std::string urdf = argc > 1 ? argv[1] : "panda.urdf";
  const std::string ee = "panda_link8";

  std::vector<std::unique_ptr<DataTransmitter>> transmitters;
    transmitters.reserve(2);
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 12, "ROBOT"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 13, "DISTANCE"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 14, "TRAJDATA"));

    auto loop_start = std::chrono::steady_clock::now();

    // ── Delay for loading web interface ──────────────────────────────────────────
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start).count() <= 6.0) {;} 
    
  // ---------------------------------------------------------------- Parameters
  const double time_final = 5.0;
  const double time_experiment = 30.0;
  const double freq = 200.0;
  const double dt = 1.0 / freq;
  const double stopping_time = 0.25;
  const double pause_after_collision = 3.75;
  const double velocity_PFL = 0.4;
  const double Qv_PFL = 0.08;
  const double HR_clearance = 0.1;
  const int n_steps = static_cast<int>(freq*time_experiment);  // MATLAB: for i=1:380

  // TODO: set to the values your MATLAB SSMPFL_franka used.
  const double Qpj = 70.0;
  const double Qpt = 1.0;

  const int N = static_cast<int>(std::lround(time_final / dt)) + 1;  // 1001
  const int Nc = static_cast<int>(std::lround(time_experiment / dt)) + 1;
  const int M = (N - 1) * 3 + 1;  // padded trajectory length

  const Eigen::Vector3d robot_start(0.7, 0.1, 0.3);
  const Eigen::Vector3d robot_end(0.1, 0.7, 0.4);

  Eigen::VectorXd q_base(7);
  q_base << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;

  RobotModel robot(urdf);

  // --------------------------------------------------- Robot trajectory (IK)
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R(1, 1) = -1.0;
  R(2, 2) = -1.0;
  Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
  T1.linear() = R;  T1.translation() = robot_start;
  T2.linear() = R;  T2.translation() = robot_end;

  Eigen::VectorXd q1, q2;
  if (!robot.ComputeIK(ee, T1, q_base, &q1)) throw std::runtime_error("IK failed for start pose");
  if (!robot.ComputeIK(ee, T2, q_base, &q2)) throw std::runtime_error("IK failed for end pose");

  // Forward (q1->q2) and backward (q2->q1) joint trajectories, padded by holding
  // the last position (velocities/accelerations stay zero, as in MATLAB).
  auto build_joint_traj = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b, Eigen::MatrixXd& q,
                              Eigen::MatrixXd& qd, Eigen::MatrixXd& qdd) {
    Traj tr = QuinticPolyTraj(a, b, time_final, dt, N);
    q = Eigen::MatrixXd::Zero(7, M);
    qd = Eigen::MatrixXd::Zero(7, M);
    qdd = Eigen::MatrixXd::Zero(7, M);
    q.leftCols(N) = tr.p;
    qd.leftCols(N) = tr.v;
    qdd.leftCols(N) = tr.a;
    for (int k = N; k < M; ++k) q.col(k) = q.col(N - 1);
  };
  Eigen::MatrixXd qf, qdf, qddf, qs, qds, qdds;
  build_joint_traj(q1, q2, qf, qdf, qddf);
  build_joint_traj(q2, q1, qs, qds, qdds);

  // Cartesian analysis: position and 6D velocity (Pinocchio: [linear; angular]).
  auto build_cart = [&](const Eigen::MatrixXd& q, const Eigen::MatrixXd& qd, Eigen::MatrixXd& p,
                        Eigen::MatrixXd& v, std::vector<double>* vnorm) {
    p = Eigen::MatrixXd::Zero(3, M);
    v = Eigen::MatrixXd::Zero(6, M);  // stays zero past N, as in MATLAB
    for (int k = 0; k < N; ++k) {
      v.col(k) = robot.ComputeJacobian(ee, q.col(k)) * qd.col(k);
      p.col(k) = robot.GetJointPose(ee, q.col(k)).translation();
      if (vnorm) vnorm->push_back(v.col(k).norm());
    }
    for (int k = N; k < M; ++k) p.col(k) = p.col(N - 1);
  };
  Eigen::MatrixXd pf, vf, ps, vs;
  std::vector<double> v_norm;
  build_cart(qf, qdf, pf, vf, &v_norm);
  build_cart(qs, qds, ps, vs, nullptr);

  double v_max = *std::max_element(v_norm.begin(), v_norm.end());
  double v_rms = 0;
  for (double x : v_norm) v_rms += x * x;
  v_rms = std::sqrt(v_rms / v_norm.size());
  std::cout << "v_max = " << v_max << "\nv_norm_base(rms) = " << v_rms << "\n";

  // ------------------------------------------------------ Human trajectory
  const double t_move = 1.25, t_pause = 2.5;
  const int Nm = static_cast<int>(std::lround(t_move * freq)) + 1;  // 251
  const int Nh = static_cast<int>(std::lround((2 * t_move + t_pause) * freq)) + 1;  // 1001

  const Eigen::Vector3d hA(0.8, 0.8, 0.3), hB(0.4, 0.4, 0.3);
  Traj hf = QuinticPolyTraj(hA, hB, t_move, dt, Nm);  // A -> B
  Traj hs = QuinticPolyTraj(hB, hA, t_move, dt, Nm);  // B -> A

  Eigen::MatrixXd p_unit(3, Nh), v_unit = Eigen::MatrixXd::Zero(3, Nh);
  for (int k = 0; k < Nh; ++k) p_unit.col(k) = hA;
  const int m = Nm - 1;  // 250: last index of first move / first of second
  p_unit.block(0, 0, 3, Nm) = hf.p;
  v_unit.block(0, 0, 3, Nm) = hf.v;
  p_unit.block(0, m, 3, Nm) = hs.p;
  v_unit.block(0, m, 3, Nm) = hs.v;

  Eigen::MatrixXd p_int = Eigen::MatrixXd::Zero(3, Nc), v_int = Eigen::MatrixXd::Zero(3, Nc);
  const int reps = static_cast<int>(std::lround(time_experiment / time_final));
  for (int j = 0; j < reps; ++j) {
    const int start = j * static_cast<int>(freq * time_final);
    p_int.block(0, start, 3, Nh) = p_unit;
    v_int.block(0, start, 3, Nh) = v_unit;
  }

  // -------------------------------------- PFL & SSM & Escape simulation
  auto t0 = std::chrono::steady_clock::now();

  std::vector<Eigen::VectorXd> qdd_real{Eigen::VectorXd::Zero(7)};
  std::vector<Eigen::VectorXd> qd_real{qdf.col(0)};
  std::vector<Eigen::VectorXd> q_real{qf.col(0)};
  std::vector<Eigen::Vector3d> p_real{pf.col(0)};
  std::vector<Eigen::Vector3d> v_real{vf.col(0).head<3>()};
  std::vector<int> flag;

  int R_STOP_collision = 0, R_STOP_computational = 0, R_CYCLES = 0;
  std::vector<int> T_TIME;
  int collision_counter = 0;
  bool collision = false, forward = true;
  int r = 1;  // MATLAB reference_time (reference column used is r, 0-based)

  for (int i = 1; i <= n_steps; ++i) {
    const int k = i - 1;  // index of the latest state (MATLAB column i)

    if (!collision) {
      const Eigen::MatrixXd& Pr = forward ? pf : ps;
      const Eigen::MatrixXd& Vr = forward ? vf : vs;
      const Eigen::MatrixXd& Qr = forward ? qf : qs;
      const int rr = std::min(r, M - 1);

      Eigen::Vector3d ro = p_int.col(k);
      Eigen::Vector3d vo = v_int.col(k);


    // ro = Eigen::Vector3d{0.6120, 0.6120, 0.3000};
    // vo = Eigen::Vector3d{0.5988, 0.5988, 0.0};

    // ro = Eigen::Vector3d{0.5, 0.5, 0.3};
    // vo = Eigen::Vector3d{0.0, 0.0, 0.0};




      const double delta =
          -(-(HR_clearance + vo.norm() * stopping_time) / stopping_time + velocity_PFL) *
          stopping_time;

      SSMPFLResult res = SSMPFL(robot, dt, stopping_time, q_real[k], qd_real[k],
                                Pr.col(rr), Vr.col(rr).head<3>(), Qr.col(rr),
                                ro, vo, delta, Qpj, Qpt, Qv_PFL);
      qdd_real.push_back(res.qdd_next);
      qd_real.push_back(res.qd_next);
      q_real.push_back(res.q_next);
      p_real.push_back(res.p_next);
      v_real.push_back(res.pd_next);
      flag.push_back(res.exitflag ? 1 : 0);
      ++r;



    std::vector<nlohmann::json> payload;
    payload.push_back(std::vector<std::array<double, 3>>{{0, 0, 0}});
    payload.push_back(std::vector<double>(res.q_next.data(), res.q_next.data() + res.q_next.size()));
    payload.push_back(std::vector<int>{});
    transmitters[0]->send_data(payload);

    payload.clear();
    payload.push_back(std::array<double, 3>{{ro[0], ro[1], ro[2]}});
    payload.push_back(std::array<double, 3>{{res.p_next[0], res.p_next[1], res.p_next[2]}});
    transmitters[1]->send_data(payload);

    Eigen::Vector3d p_r;
    p_r = robot.GetJointPose("panda_link8", Qr.col(rr)).translation().transpose();

    payload.clear();
    payload.push_back(std::vector<double>(res.p_next.data(), res.p_next.data() + res.p_next.size()));
    payload.push_back(std::vector<double>(p_r.data(), p_r.data() + p_r.size()));
    transmitters[2]->send_data(payload);





      if (res.exitflag) {
        // if (!CollisionFree(robot, q_real[k], ro, HR_clearance)) {
        //   ++R_STOP_collision;
        //   collision = true;
        //   collision_counter = 0;
        // }
        if (forward) {
          if ((p_real[k] - pf.col(N - 1)).norm() <= 0.01) {
            r = 1;
            forward = false;
          }
        } else {
          if ((p_real[k] - ps.col(N - 1)).norm() <= 0.01) {
            r = 1;
            forward = true;
            ++R_CYCLES;
            T_TIME.push_back(i);
          }
        }
      } else {
        collision = true;
        collision_counter = 0;
        ++R_STOP_computational;
      }
    } else {
      ++collision_counter;
      qdd_real.push_back(Eigen::VectorXd::Zero(7));
      qd_real.push_back(Eigen::VectorXd::Zero(7));
      // NOTE: faithful to MATLAB `q_real(:,end-1)` / `p_real(:,end-1)`, which
      // repeats the sample *before* the latest one. Use [k] instead if you
      // actually intend the robot to hold its current pose during the pause.
      const int prev = k > 0 ? k - 1 : k;
      q_real.push_back(q_real[prev]);
      p_real.push_back(p_real[prev]);
      v_real.push_back(Eigen::Vector3d::Zero());
      if (collision_counter > pause_after_collision * freq) collision = false;
    }
  }

  auto t1 = std::chrono::steady_clock::now();

  std::cout << "R_STOP_computational = " << R_STOP_computational
            << "\nR_STOP_collision     = " << R_STOP_collision
            << "\nR_CYCLES             = " << R_CYCLES << "\n"
            << "elapsed: " << std::chrono::duration<double>(t1 - t0).count() << " s\n";

  // Replacement for plot3(): dump the paths for external plotting.
  WriteCsv("path_real.csv", p_real);
  WriteCsv("path_human.csv", p_int);
  WriteCsv("path_nominal_forward.csv", pf);
  WriteCsv("path_nominal_backward.csv", ps);
  std::cout << "wrote path_*.csv\n";


  PlotPaths(p_real, p_int, pf, ps);


  return 0;
}
#include "robot_model.hpp"

#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>


// ─────────────────────────────────────────────────────────────────────────────
// Load the model from a URDF file
// ─────────────────────────────────────────────────────────────────────────────
RobotModel::RobotModel(const std::string& urdf_path) {
    std::cout << "==urdf_path = " << urdf_path << std::endl;
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);
}

// ─────────────────────────────────────────────────────────────────────────────
// Resolve a frame name to its Pinocchio index, throwing if absent
// ─────────────────────────────────────────────────────────────────────────────
pinocchio::FrameIndex RobotModel::GetFrameIndexOrThrow(
    const std::string& frame_name) const {
  if (!model_.existFrame(frame_name)) {
    throw std::runtime_error("Frame not found in URDF: " + frame_name);
  }
  return model_.getFrameId(frame_name);
}

// ─────────────────────────────────────────────────────────────────────────────
// Update internal kinematic data for configuration q
// ─────────────────────────────────────────────────────────────────────────────
void RobotModel::ComputeFK(const Eigen::VectorXd& q) const {
  if (q.size() != model_.nq) {
    throw std::runtime_error("Joint vector size does not match model DOF");
  }
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
}

// ─────────────────────────────────────────────────────────────────────────────
// Pose of a named frame at configuration q
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Isometry3d RobotModel::GetJointPose(const std::string& frame_name,
                                            const Eigen::VectorXd& q) const {
  ComputeFK(q);
  return GetJointPose(frame_name);
}

// ─────────────────────────────────────────────────────────────────────────────
// Pose of a named frame using the last configuration passed to ComputeFK
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Isometry3d RobotModel::GetJointPose(
    const std::string& frame_name) const {
  const pinocchio::FrameIndex frame_id = GetFrameIndexOrThrow(frame_name);
  const pinocchio::SE3& placement = data_.oMf[frame_id];

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = placement.rotation();
  pose.translation() = placement.translation();
  return pose;
}

// ─────────────────────────────────────────────────────────────────────────────
// Geometric Jacobian of a named frame at configuration q
// ─────────────────────────────────────────────────────────────────────────────
Eigen::MatrixXd RobotModel::ComputeJacobian(const std::string& frame_name,
                                             const Eigen::VectorXd& q) const {
  if (q.size() != model_.nq) {
    throw std::runtime_error("Joint vector size does not match model DOF");
  }
  const pinocchio::FrameIndex frame_id = GetFrameIndexOrThrow(frame_name);

  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  Eigen::MatrixXd J(6, model_.nv);
  J.setZero();
  pinocchio::computeFrameJacobian(
      model_, data_, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
  return J;
}

// ─────────────────────────────────────────────────────────────────────────────
// Finite-difference derivative of the frame Jacobian at configuration q
// ─────────────────────────────────────────────────────────────────────────────
Eigen::MatrixXd RobotModel::ComputeDerivativeJacobian(const std::string& frame_name,
                                             const Eigen::VectorXd& q) const {
    constexpr double eps = 0.000001;

    Eigen::MatrixXd J = ComputeJacobian(frame_name, q);

    Eigen::VectorXd q_e = (q.array() + eps).matrix();
    Eigen::MatrixXd J_e = ComputeJacobian(frame_name, q_e);

    return (J_e - J)/eps;
}

// ─────────────────────────────────────────────────────────────────────────────
// Damped-least-squares inverse kinematics to reach a target pose
// ─────────────────────────────────────────────────────────────────────────────
bool RobotModel::ComputeIK(const std::string& frame_name,
                            const Eigen::Isometry3d& target_pose,
                            const Eigen::VectorXd& q_init,
                            Eigen::VectorXd* q_result,
                            double eps,
                            int max_iters,
                            double damping) const {
  const pinocchio::FrameIndex frame_id = GetFrameIndexOrThrow(frame_name);

  pinocchio::SE3 target;
  target.rotation() = target_pose.linear();
  target.translation() = target_pose.translation();

  // ── Local working copies ────────────────────────────────────────────────
  pinocchio::Data data(model_);
  Eigen::VectorXd q = q_init;

  Eigen::MatrixXd J(6, model_.nv);
  bool converged = false;

  for (int i = 0; i < max_iters; ++i) {
    pinocchio::forwardKinematics(model_, data, q);
    pinocchio::updateFramePlacements(model_, data);

    // ── Pose error twist ──────────────────────────────────────────────────
    const pinocchio::SE3 current_pose = data.oMf[frame_id];
    const pinocchio::Motion err_motion = pinocchio::log6(current_pose.actInv(target));
    const Eigen::Matrix<double, 6, 1> err = err_motion.toVector();

    if (err.norm() < eps) {
      converged = true;
      break;
    }

    J.setZero();
    pinocchio::computeFrameJacobian(model_, data, q, frame_id,
                                     pinocchio::LOCAL, J);

    // ── Damped-least-squares update ───────────────────────────────────────
    Eigen::MatrixXd JJt =
        J * J.transpose() + damping * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(err);

    Eigen::VectorXd q_next(q.size());
    pinocchio::integrate(model_, q, dq, q_next);
    q = q_next;
  }

  if (converged && q_result != nullptr) {
    *q_result = q;
  }
  return converged;
}

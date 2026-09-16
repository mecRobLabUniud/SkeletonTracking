// robot_model.cpp
#include "robot_model.hpp"

#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>


RobotModel::RobotModel(const std::string& urdf_path) {
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);
}

pinocchio::FrameIndex RobotModel::GetFrameIndexOrThrow(
    const std::string& frame_name) const {
  if (!model_.existFrame(frame_name)) {
    throw std::runtime_error("Frame not found in URDF: " + frame_name);
  }
  return model_.getFrameId(frame_name);
}

void RobotModel::ComputeFK(const Eigen::VectorXd& q) const {
  if (q.size() != model_.nq) {
    throw std::runtime_error("Joint vector size does not match model DOF");
  }
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
}

Eigen::Isometry3d RobotModel::GetJointPose(const std::string& frame_name,
                                            const Eigen::VectorXd& q) const {
  ComputeFK(q);
  return GetJointPose(frame_name);
}

Eigen::Isometry3d RobotModel::GetJointPose(
    const std::string& frame_name) const {
  const pinocchio::FrameIndex frame_id = GetFrameIndexOrThrow(frame_name);
  const pinocchio::SE3& placement = data_.oMf[frame_id];

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = placement.rotation();
  pose.translation() = placement.translation();
  return pose;
}

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

Eigen::MatrixXd RobotModel::ComputeDerivativeJacobian(const std::string& frame_name,
                                             const Eigen::VectorXd& q) const {
    constexpr double eps = 0.0000001;

    Eigen::MatrixXd J = ComputeJacobian(frame_name, q);

    Eigen::VectorXd q_e = (q.array() + eps).matrix();
    Eigen::MatrixXd J_e = ComputeJacobian(frame_name, q_e);

    return (J_e - J) / eps;
}
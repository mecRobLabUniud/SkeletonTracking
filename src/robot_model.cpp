// robot_model.cpp
#include "robot_model.hpp"

#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/explog.hpp>


/*
    const std::string urdf_path = c_dir + "/src/urdf/panda.urdf";

    RobotModel robot(urdf_path);

    Eigen::VectorXd q(7);
    q << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;  // Panda "ready" pose
    
    ee_pose = robot.GetJointPose("panda_link8", q);
    std::cout << "EE position: " << ee_pose.translation().transpose()
                << std::endl; 


    // Jacobian at same q
    Eigen::MatrixXd J = robot.ComputeJacobian("panda_link8", q);
    std::cout << "Jacobian (6x7):\n" << J << std::endl;

    // Inverse kinematics: try to reach a slightly perturbed target
    Eigen::Isometry3d target = ee_pose;
    target.translation().z() += 0.05;

    Eigen::VectorXd q_solution;
    bool ok = robot.ComputeIK("panda_link8", target, q, &q_solution);
    if (ok) {
        std::cout << "IK solution: " << q_solution.transpose() << std::endl;
    } else {
        std::cout << "IK did not converge" << std::endl;
    }
*/


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

  // Work on local copies so this method stays const w.r.t. the class.
  pinocchio::Data data(model_);
  Eigen::VectorXd q = q_init;

  Eigen::MatrixXd J(6, model_.nv);
  bool converged = false;

  for (int i = 0; i < max_iters; ++i) {
    pinocchio::forwardKinematics(model_, data, q);
    pinocchio::updateFramePlacements(model_, data);

    // Error twist between current and target pose, in the frame's local
    // coordinates (standard formulation for Pinocchio-based IK loops).
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

    // Damped least squares: dq = J^T (J J^T + lambda*I)^-1 * err
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
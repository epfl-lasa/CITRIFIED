#pragma once

#include <franka_lwi/franka_lwi_communication_protocol.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>
#include <state_representation/robot/JointTorques.hpp>

namespace frankalwi::utils {

inline void toCartesianPose(const frankalwi::proto::StateMessage<7>& state, state_representation::CartesianPose& pose) {
  pose.set_pose(Eigen::Map<Eigen::MatrixXd>(state.eePose.array().data(), 7, 1));
}

inline void
toCartesianState(const frankalwi::proto::StateMessage<7>& state, state_representation::CartesianState& cartesianState) {
  cartesianState.set_pose(Eigen::Map<Eigen::MatrixXd>(state.eePose.array().data(), 7, 1));
  cartesianState.set_twist(Eigen::Map<Eigen::MatrixXd>(state.eeTwist.array().data(), 6, 1));
  cartesianState.set_wrench(Eigen::Map<Eigen::MatrixXd>(state.eeWrench.array().data(), 6, 1));
}

inline void toJointState(const frankalwi::proto::StateMessage<7>& state, state_representation::JointState& jointState) {
  jointState.set_positions(Eigen::Matrix<double, 7, 1>(state.jointPosition.array().data()));
  jointState.set_velocities(Eigen::Matrix<double, 7, 1>(state.jointVelocity.array().data()));
  jointState.set_torques(Eigen::Matrix<double, 7, 1>(state.jointTorque.array().data()));
}

inline void toJacobian(const frankalwi::proto::Jacobian<7>& stateJacobian, state_representation::Jacobian& jacobian) {
  frankalwi::proto::Jacobian<7> copy = stateJacobian;
  jacobian.set_data(Eigen::Map<Eigen::Matrix<double, 6, 7>>(copy.data()));
}

inline void
fromJointTorque(const state_representation::JointTorques& torques, frankalwi::proto::CommandMessage<7>& command) {
  Eigen::MatrixXd::Map(command.jointTorque.data.data(), 7, 1) = torques.data().array();
}

}
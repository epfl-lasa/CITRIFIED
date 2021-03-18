#pragma once

#include <franka_lwi/franka_lwi_communication_protocol.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>

namespace frankalwi::utils {

void toCartesianPose(frankalwi::proto::StateMessage<7>& state, state_representation::CartesianPose& pose) {
  pose.set_position(Eigen::Vector3d(frankalwi::proto::vec3DToArray(state.eePose.position).data()));
  pose.set_orientation(Eigen::Quaterniond(state.eePose.orientation.w, state.eePose.orientation.x, state.eePose.orientation.y, state.eePose.orientation.z));
}

void toCartesianState(const frankalwi::proto::StateMessage<7>& state, state_representation::CartesianState& cartesianState) {
  cartesianState.set_position(Eigen::Vector3d(frankalwi::proto::vec3DToArray(state.eePose.position).data()));
  cartesianState.set_orientation(Eigen::Quaterniond(state.eePose.orientation.w, state.eePose.orientation.x, state.eePose.orientation.y, state.eePose.orientation.z));
  cartesianState.set_linear_velocity(Eigen::Vector3d(frankalwi::proto::vec3DToArray(state.eeTwist.linear).data()));
  cartesianState.set_angular_velocity(Eigen::Vector3d(frankalwi::proto::vec3DToArray(state.eeTwist.angular).data()));
  cartesianState.set_force(Eigen::Vector3d(frankalwi::proto::vec3DToArray(state.eeWrench.angular).data()));
  cartesianState.set_torque(Eigen::Vector3d(frankalwi::proto::vec3DToArray(state.eeWrench.angular).data()));
}

void toJointState(const frankalwi::proto::StateMessage<7>& state, state_representation::JointState& jointState) {
  jointState.set_positions(Eigen::VectorXd(state.jointPosition.data.data()));
  jointState.set_velocities(Eigen::VectorXd(state.jointVelocity.data.data()));
  jointState.set_torques(Eigen::VectorXd(state.jointTorque.data.data()));
}

void toJacobian(const frankalwi::proto::Jacobian<7>& stateJacobian, state_representation::Jacobian& jacobian) {
  frankalwi::proto::Jacobian<7> copy = stateJacobian;
  jacobian.set_data(Eigen::Map<Eigen::Matrix<double, 6, 7>>(copy.data()));
}

}
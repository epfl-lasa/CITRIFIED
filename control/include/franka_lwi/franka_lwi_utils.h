#pragma once

#include <franka_lwi/franka_lwi_communication_protocol.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointState.hpp>

namespace frankalwi::utils {

void poseToState(frankalwi::proto::StateMessage<7>& state, state_representation::CartesianPose& pose) {
  Eigen::Vector3d position(frankalwi::proto::vec3DToArray(state.eePose.position).data());
  Eigen::Quaterniond orientation
      (state.eePose.orientation.w, state.eePose.orientation.x, state.eePose.orientation.y, state.eePose.orientation.z);
  state_representation::CartesianPose newPose(pose.get_name(), position, orientation, pose.get_reference_frame());
  pose = newPose;
}

}
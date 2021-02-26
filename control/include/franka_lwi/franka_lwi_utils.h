#pragma once

#include <franka_lwi/franka_lwi_communication_protocol.h>
#include <state_representation/Space/Cartesian/CartesianPose.hpp>

namespace frankalwi::proto {

void poseFromState(frankalwi::proto::StateMessage<7>& state, StateRepresentation::CartesianPose& pose) {
  Eigen::Vector3d position(frankalwi::proto::vec3DToArray(state.eePose.position).data());
  Eigen::Quaterniond orientation
      (state.eePose.orientation.w, state.eePose.orientation.x, state.eePose.orientation.y, state.eePose.orientation.z);
  StateRepresentation::CartesianPose newPose(pose.get_reference_frame(), position, orientation);
  pose = newPose;
}

}
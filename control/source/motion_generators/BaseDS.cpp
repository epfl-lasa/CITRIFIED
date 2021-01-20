#include "motion_generators/BaseDS.h"

namespace motiongenerator {

BaseDS::BaseDS() :
    currentPose(StateRepresentation::CartesianPose::Identity("world")) {

}

void BaseDS::poseFromState(const frankalwi::proto::StateMessage<7>& state, StateRepresentation::CartesianPose& pose) {
  Eigen::Vector3d position
      (state.eePose.position.x, state.eePose.position.y, state.eePose.position.z);
  Eigen::Quaterniond orientation
      (state.eePose.orientation.w, state.eePose.orientation.x, state.eePose.orientation.y, state.eePose.orientation.z);
  StateRepresentation::CartesianPose newPose(pose.get_reference_frame(), position, orientation);
  pose = newPose;
}

std::vector<double> BaseDS::clampTwist(StateRepresentation::CartesianTwist twist) const {
  twist.clamp(maxLinearSpeed, maxAngularSpeed, minLinearSpeed, minAngularSpeed);
  std::vector<double> desiredVelocity = {
      twist.get_linear_velocity().x(),
      twist.get_linear_velocity().y(),
      twist.get_linear_velocity().z(),
      twist.get_angular_velocity().x(),
      twist.get_angular_velocity().y(),
      twist.get_angular_velocity().z()
  };

  return desiredVelocity;
}

void BaseDS::updateCurrentPose(const frankalwi::proto::StateMessage<7>& state) {
  poseFromState(state, currentPose);
}

}
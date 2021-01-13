#include "motion_generators/BaseDS.h"

namespace motiongenerator {

BaseDS::BaseDS() :
    currentPose(StateRepresentation::CartesianPose::Identity("world")) {

}

void BaseDS::poseFromState(StateRepresentation::CartesianPose& pose, frankalwi::proto::StateMessage<7> state) {
  pose.set_position(state.eePose.position.x, state.eePose.position.y, state.eePose.position.z);
  pose.set_orientation(std::vector<double>{1, 0, 0, 0});
  pose.set_orientation(std::vector<double>{
      state.eePose.orientation.w, state.eePose.orientation.x, state.eePose.orientation.y, state.eePose.orientation.z
  });
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

void BaseDS::updateCurrentPose(frankalwi::proto::StateMessage<7> state) {
  poseFromState(currentPose, state);
}

}
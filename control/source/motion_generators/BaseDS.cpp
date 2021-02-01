#include "motion_generators/BaseDS.h"

namespace motion_generator {

BaseDS::BaseDS() :
    currentPose(StateRepresentation::CartesianPose::Identity("world")) {

}

StateRepresentation::CartesianTwist BaseDS::clampTwist(StateRepresentation::CartesianTwist& twist) const {
  twist.clamp(maxLinearSpeed, maxAngularSpeed, minLinearSpeed, minAngularSpeed);
  return twist;
}

void BaseDS::updateCurrentPose(const StateRepresentation::CartesianPose& pose) {
  currentPose = pose;
}

}
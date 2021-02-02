#include "motion_generators/BaseDS.h"

namespace motion_generator {

BaseDS::BaseDS() :
    currentPose(StateRepresentation::CartesianPose::Identity("world")) {}

void BaseDS::clampTwist(StateRepresentation::CartesianTwist& twist) const {
  twist.clamp(maxLinearSpeed, maxAngularSpeed, minLinearSpeed, minAngularSpeed);
}

void BaseDS::updateCurrentPose(const StateRepresentation::CartesianPose& pose) {
  currentPose = pose;
}

}
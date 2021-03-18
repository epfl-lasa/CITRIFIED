#include "motion_generators/BaseDS.h"

namespace motion_generator {

BaseDS::BaseDS() :
    currentPose(state_representation::CartesianPose::Identity("world")) {}

void BaseDS::clampTwist(state_representation::CartesianTwist& twist) const {
  twist.clamp(maxLinearSpeed, maxAngularSpeed, minLinearSpeed, minAngularSpeed);
}

void BaseDS::updateCurrentPose(const state_representation::CartesianPose& pose) {
  currentPose = pose;
}

}
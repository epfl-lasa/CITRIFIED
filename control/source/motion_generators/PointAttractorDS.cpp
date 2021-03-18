#include "motion_generators/PointAttractorDS.h"

namespace motion_generator {

PointAttractor::PointAttractor() :
    targetPose(state_representation::CartesianPose::Identity("world")),
    linearDS(targetPose) {
}

PointAttractor::PointAttractor(const state_representation::CartesianPose& pose) : targetPose(pose), linearDS(pose) {
}

void PointAttractor::setTargetPose(const state_representation::CartesianPose& pose) {
  targetPose = pose;
  linearDS.set_attractor(targetPose);
}

void PointAttractor::setTargetPosition(state_representation::CartesianPose pose) {
  pose.set_orientation(targetPose.get_orientation());
  setTargetPose(pose);
}

void PointAttractor::setTargetOrientation(state_representation::CartesianPose pose) {
  pose.set_position(targetPose.get_position());
  setTargetPose(pose);
}

state_representation::CartesianTwist PointAttractor::getTwist(const state_representation::CartesianPose& pose) {
  updateCurrentPose(pose);
  return getTwist();
}

state_representation::CartesianTwist PointAttractor::getTwist() {
  state_representation::CartesianTwist twist = linearDS.evaluate(currentPose);
  clampTwist(twist);
  return twist;
}

}
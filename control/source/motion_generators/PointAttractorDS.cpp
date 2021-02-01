#include "motion_generators/PointAttractorDS.h"

namespace motion_generator {

PointAttractor::PointAttractor() :
    targetPose(StateRepresentation::CartesianPose::Identity("world")),
    linearDS(targetPose) {
}

PointAttractor::PointAttractor(const StateRepresentation::CartesianPose& pose) : targetPose(pose), linearDS(pose) {
}

void PointAttractor::setTargetPose(const StateRepresentation::CartesianPose& pose) {
  targetPose = pose;
  linearDS.set_attractor(targetPose);
}

void PointAttractor::setTargetPosition(StateRepresentation::CartesianPose pose) {
  pose.set_orientation(targetPose.get_orientation());
  setTargetPose(pose);
}

void PointAttractor::setTargetOrientation(StateRepresentation::CartesianPose pose) {
  pose.set_position(targetPose.get_position());
  setTargetPose(pose);
}

StateRepresentation::CartesianTwist PointAttractor::getTwist(StateRepresentation::CartesianPose& pose) {
  updateCurrentPose(pose);
  return getTwist();
}

StateRepresentation::CartesianTwist PointAttractor::getTwist() {
  StateRepresentation::CartesianTwist twist = linearDS.evaluate(currentPose);
  return clampTwist(twist);
}

}
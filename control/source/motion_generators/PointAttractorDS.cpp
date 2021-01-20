#include "motion_generators/PointAttractorDS.h"

namespace motiongenerator {

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

void PointAttractor::setTargetPose(frankalwi::proto::StateMessage<7> state) {
  StateRepresentation::CartesianPose pose;
  poseFromState(state, pose);
  setTargetPose(pose);
}

void PointAttractor::setTargetPosition(StateRepresentation::CartesianPose pose) {
  pose.set_orientation(targetPose.get_orientation());
  setTargetPose(pose);
}
void PointAttractor::setTargetPosition(frankalwi::proto::StateMessage<7> state) {
  StateRepresentation::CartesianPose pose;
  poseFromState(state, pose);
  setTargetPosition(pose);
}

void PointAttractor::setTargetOrientation(StateRepresentation::CartesianPose pose) {
  pose.set_position(targetPose.get_position());
  setTargetPose(pose);
}
void PointAttractor::setTargetOrientation(frankalwi::proto::StateMessage<7> state) {
  StateRepresentation::CartesianPose pose;
  poseFromState(state, pose);
  setTargetOrientation(pose);
}

std::vector<double> PointAttractor::getTwist(frankalwi::proto::StateMessage<7> state) {
  updateCurrentPose(state);
  return getTwist();
}

std::vector<double> PointAttractor::getTwist() {
  StateRepresentation::CartesianTwist twist = linearDS.evaluate(currentPose);
  return clampTwist(twist);
}

}
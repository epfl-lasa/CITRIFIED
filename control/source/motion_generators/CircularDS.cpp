#include "motion_generators/CircularDS.h"

namespace motiongenerator {

CircleDS::CircleDS() :
    center(StateRepresentation::CartesianPose::Identity("world")),
    circularDS(center) {
}

CircleDS::CircleDS(const StateRepresentation::CartesianPose& pose) :
    center(pose),
    circularDS(center) {
}

CircleDS::CircleDS(const StateRepresentation::CartesianPose& pose, double radius) :
    center(pose),
    circularDS(center) {
  circularDS.set_radius(radius);
}

std::vector<double> CircleDS::getTwist(frankalwi::proto::StateMessage<7> state) {
  updateCurrentPose(state);
  return getTwist();
}
std::vector<double> CircleDS::getTwist() {
  StateRepresentation::CartesianTwist twist = circularDS.evaluate(currentPose);

  return clampTwist(twist);
}

}
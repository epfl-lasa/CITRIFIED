#include "motion_generators/CircularDS.h"

namespace motion_generator {

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

StateRepresentation::CartesianTwist CircleDS::getTwist(const StateRepresentation::CartesianPose& pose) {
  updateCurrentPose(pose);
  return getTwist();
}
StateRepresentation::CartesianTwist CircleDS::getTwist() {
  StateRepresentation::CartesianTwist twist = circularDS.evaluate(currentPose);
  clampTwist(twist);
  return twist;
}

}
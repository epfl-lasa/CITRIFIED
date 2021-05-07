#include "motion_generators/CircularDS.h"

namespace motion_generator {

CircleDS::CircleDS() :
    center(state_representation::CartesianPose::Identity("world")),
    circularDS(center) {
}

CircleDS::CircleDS(const state_representation::CartesianPose& pose) :
    center(pose),
    circularDS(center) {
}

CircleDS::CircleDS(const state_representation::CartesianPose& pose, double radius) :
    center(pose),
    circularDS(center) {
  circularDS.set_radius(radius);
}

state_representation::CartesianTwist CircleDS::getTwist(const state_representation::CartesianPose& pose) {
  updateCurrentPose(pose);
  return getTwist();
}
state_representation::CartesianTwist CircleDS::getTwist() {
  state_representation::CartesianTwist twist = circularDS.evaluate(currentPose);
  clampTwist(twist);
  return twist;
}

}
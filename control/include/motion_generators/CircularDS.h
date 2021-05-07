
#pragma once

#include "BaseDS.h"

#include <dynamical_systems/Circular.hpp>

namespace motion_generator {

class CircleDS : public BaseDS {
public:
  CircleDS();
  explicit CircleDS(const state_representation::CartesianPose& pose);
  CircleDS(const state_representation::CartesianPose& pose, double radius);

  state_representation::CartesianTwist getTwist(const state_representation::CartesianPose& pose) override;
  state_representation::CartesianTwist getTwist();

  state_representation::CartesianPose center;
  dynamical_systems::Circular circularDS;
};

}

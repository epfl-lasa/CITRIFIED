
#pragma once

#include "BaseDS.h"

#include <dynamical_systems/Circular.hpp>

namespace motion_generator {

class CircleDS : public BaseDS {
public:
  CircleDS();
  explicit CircleDS(const StateRepresentation::CartesianPose& pose);
  CircleDS(const StateRepresentation::CartesianPose& pose, double radius);

  StateRepresentation::CartesianTwist getTwist(StateRepresentation::CartesianPose& pose) override;
  StateRepresentation::CartesianTwist getTwist();

  StateRepresentation::CartesianPose center;
  DynamicalSystems::Circular circularDS;
};

}

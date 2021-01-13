
#pragma once

#include "BaseDS.h"
#include <dynamical_systems/Circular.hpp>

namespace motiongenerator {

class CircleDS : public BaseDS {
public:
  CircleDS();
  explicit CircleDS(const StateRepresentation::CartesianPose& pose);
  CircleDS(const StateRepresentation::CartesianPose& pose, double radius);

  std::vector<double> getTwist(frankalwi::proto::StateMessage<7> state) override;
  std::vector<double> getTwist();

  StateRepresentation::CartesianPose center;
  DynamicalSystems::Circular circularDS;
};

}

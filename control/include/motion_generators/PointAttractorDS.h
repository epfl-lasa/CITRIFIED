
#pragma once

#include "BaseDS.h"

#include <dynamical_systems/Linear.hpp>

namespace motion_generator {

class PointAttractor : public BaseDS {
public:
  PointAttractor();
  explicit PointAttractor(const StateRepresentation::CartesianPose& pose);

  void setTargetPose(const StateRepresentation::CartesianPose& pose);
  void setTargetPosition(StateRepresentation::CartesianPose pose);
  void setTargetOrientation(StateRepresentation::CartesianPose pose);

  StateRepresentation::CartesianTwist getTwist(const StateRepresentation::CartesianPose& pose) override;
  StateRepresentation::CartesianTwist getTwist();

  StateRepresentation::CartesianPose targetPose;
  DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS;
};

}

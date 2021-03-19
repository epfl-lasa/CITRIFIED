
#pragma once

#include "BaseDS.h"

#include <dynamical_systems/Linear.hpp>

namespace motion_generator {

class PointAttractor : public BaseDS {
public:
  PointAttractor();
  explicit PointAttractor(const state_representation::CartesianPose& pose);

  void setTargetPose(const state_representation::CartesianPose& pose);
  void setTargetPosition(state_representation::CartesianPose pose);
  void setTargetOrientation(state_representation::CartesianPose pose);

  state_representation::CartesianTwist getTwist(const state_representation::CartesianPose& pose) override;
  state_representation::CartesianTwist getTwist();

  state_representation::CartesianPose targetPose;
  dynamical_systems::Linear<state_representation::CartesianState> linearDS;
};

}

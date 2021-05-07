
#pragma once

#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

namespace motion_generator {

class BaseDS {
public:
  BaseDS();
  void updateCurrentPose(const state_representation::CartesianPose& pose);

  virtual state_representation::CartesianTwist getTwist(const state_representation::CartesianPose& pose) = 0;

  state_representation::CartesianPose currentPose;
  // TODO find a solution for these hardcoded limits
  double maxLinearSpeed = 0.25;
  double maxAngularSpeed = 0.75;
  double minLinearSpeed = 1e-3;
  double minAngularSpeed = 1e-3;

protected:
  void clampTwist(state_representation::CartesianTwist& twist) const;
};

}


#pragma once

#include <state_representation/Space/Cartesian/CartesianPose.hpp>
#include <state_representation/Space/Cartesian/CartesianTwist.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

namespace motion_generator {

class BaseDS {
public:
  BaseDS();
  void updateCurrentPose(const StateRepresentation::CartesianPose& pose);

  virtual StateRepresentation::CartesianTwist getTwist(StateRepresentation::CartesianPose& pose) = 0;

  StateRepresentation::CartesianPose currentPose;
  // TODO find a solution for these hardcoded limits
  double maxLinearSpeed = 0.25;
  double maxAngularSpeed = 0.75;
  double minLinearSpeed = 1e-3;
  double minAngularSpeed = 1e-3;

protected:
  StateRepresentation::CartesianTwist clampTwist(StateRepresentation::CartesianTwist& twist) const;
};

}

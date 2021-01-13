
#pragma once

#include "franka_lwi/franka_lwi_communication_protocol.h"
#include <state_representation/Space/Cartesian/CartesianState.hpp>
#include <state_representation/Space/Cartesian/CartesianPose.hpp>
#include <state_representation/Space/Cartesian/CartesianTwist.hpp>

namespace motiongenerator {

class BaseDS {
public:
  BaseDS();
  static void poseFromState(StateRepresentation::CartesianPose& pose, frankalwi::proto::StateMessage<7> state);
  void updateCurrentPose(frankalwi::proto::StateMessage<7> state);

  virtual std::vector<double> getTwist(frankalwi::proto::StateMessage<7> state) = 0;

  StateRepresentation::CartesianPose currentPose;
  double maxLinearSpeed = 0.25;
  double maxAngularSpeed = 0.75;
  double minLinearSpeed = 1e-3;
  double minAngularSpeed = 1e-3;

protected:
  std::vector<double> clampTwist(StateRepresentation::CartesianTwist twist) const;
};

}

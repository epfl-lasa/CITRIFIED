
#pragma once

#include "BaseDS.h"
#include "dynamical_systems/Linear.hpp"

namespace motiongenerator {

class PointAttractor : public BaseDS {
public:
  PointAttractor();
  explicit PointAttractor(const StateRepresentation::CartesianPose& pose);

  void setTargetPose(const StateRepresentation::CartesianPose& pose);
  void setTargetPosition(StateRepresentation::CartesianPose pose);
  void setTargetOrientation(StateRepresentation::CartesianPose pose);

  void setTargetPose(frankalwi::proto::StateMessage<7> state);
  void setTargetPosition(frankalwi::proto::StateMessage<7> state);
  void setTargetOrientation(frankalwi::proto::StateMessage<7> state);

  std::vector<double> getTwist(frankalwi::proto::StateMessage<7> state) override;
  std::vector<double> getTwist();

  StateRepresentation::CartesianPose targetPose;
  DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS;
};

}

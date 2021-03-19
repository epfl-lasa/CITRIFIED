#pragma once

#include <controllers/Controller.hpp>
#include <passive-ds-control/passive_ds_controller.h>

namespace controllers {
class TwistController : public Controller<state_representation::CartesianState> {
public:
  TwistController(double d0, double d1, double ak, double ad);

  state_representation::CartesianState compute_command(const state_representation::CartesianState& desired_state,
                                                       const state_representation::CartesianState& feedback_state) override;

  state_representation::JointState compute_command(const state_representation::CartesianState& desired_state,
                                                   const state_representation::CartesianState& feedback_state,
                                                   const state_representation::Jacobian& jacobian) override;

  double max_force = 50;
  double max_torque = 50;
private:
  std::unique_ptr<PassiveDSController> controller_;
  double d0_;
  double d1_;
  double ak_;
  double ad_;
};
}


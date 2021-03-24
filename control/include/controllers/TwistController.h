#pragma once

#include <controllers/Controller.hpp>
#include <controllers/impedance/Dissipative.hpp>

namespace controllers {
class TwistController : public Controller<state_representation::CartesianState> {
public:
  TwistController(double d0, double d1, double ak, double ad);

  state_representation::CartesianState compute_command(const state_representation::CartesianState& desired_state,
                                                       const state_representation::CartesianState& feedback_state) override;

  state_representation::JointState compute_command(const state_representation::CartesianState& desired_state,
                                                   const state_representation::CartesianState& feedback_state,
                                                   const state_representation::Jacobian& jacobian) override;

  void set_linear_damping(double d0, double d1);
  double get_linear_damping(int index) const;

  double max_force = 50;
  double max_torque = 50;
  double angular_stiffness;
  double angular_damping;
private:
  controllers::impedance::Dissipative<state_representation::CartesianState> linear_dissipative_ctrl_;
  double d0_;
  double d1_;
};
}


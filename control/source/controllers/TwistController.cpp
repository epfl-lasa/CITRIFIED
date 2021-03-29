
#include "controllers/TwistController.h"

namespace controllers {

TwistController::TwistController(double d0, double d1, double ak, double ad) :
    d0_(d0), d1_(d1), angular_stiffness(ak), angular_damping(ad), linear_dissipative_ctrl_(controllers::impedance::ComputationalSpaceType::LINEAR) {
  set_linear_damping(d0, d1);
}

void TwistController::set_linear_damping(double d0, double d1) {
  d0_ = d0;
  d1_ = d1;
  Eigen::VectorXd damping(6);
  damping << d0, d1, d1, 0, 0, 0;
  linear_dissipative_ctrl_.set_damping_eigenvalues(damping);
}

double TwistController::get_linear_damping(int index) const {
  if (index == 0) {
    return d0_;
  } else {
    return d1_;
  }
}
state_representation::CartesianState TwistController::compute_command(const state_representation::CartesianState& desired_state,
                                                                      const state_representation::CartesianState& feedback_state) {
  state_representation::CartesianTwist desired(desired_state.get_name(), desired_state.get_linear_velocity(), desired_state.get_reference_frame());
  state_representation::CartesianTwist feedback(feedback_state.get_name(), feedback_state.get_linear_velocity(), feedback_state.get_reference_frame());
  state_representation::CartesianWrench command = linear_dissipative_ctrl_.compute_command(desired, feedback);

  // Find angular displacement from desired velocity
  auto angular_displacement = std::chrono::seconds(1) * state_representation::CartesianTwist(desired_state);

  // Feed-forward angular displacement and damp the angular velocity error
  Eigen::Vector3d torque = angular_stiffness * angular_displacement.get_orientation().vec() - angular_damping * (feedback_state.get_angular_velocity() - desired_state.get_angular_velocity());
  command.set_torque(torque);

  command.clamp(max_force, max_torque);
  return command;
}

state_representation::JointState TwistController::compute_command(const state_representation::CartesianState& desired_state,
                                                                  const state_representation::CartesianState& feedback_state,
                                                                  const state_representation::Jacobian& jacobian) {
  auto wrench = state_representation::CartesianWrench(compute_command(desired_state, feedback_state));
  return jacobian.transpose() * wrench;
}
}
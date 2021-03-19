
#include "controllers/TwistController.h"

namespace controllers {

TwistController::TwistController(double d0, double d1, double ak, double ad) :
  d0_(d0), d1_(d1), ak_(ak), ad_(ad) {
  controller_ = std::make_unique<PassiveDSController>(3, d0, d1, 1, 1);
}

state_representation::CartesianState TwistController::compute_command(const state_representation::CartesianState& desired_state,
                                                                      const state_representation::CartesianState& feedback_state) {
  auto command = state_representation::CartesianWrench(feedback_state);

  Vec stateVel(3), commandVel(3);
  stateVel << feedback_state.get_linear_velocity().x(), feedback_state.get_linear_velocity().y(), feedback_state.get_linear_velocity().z();
  commandVel << desired_state.get_linear_velocity().x(), desired_state.get_linear_velocity().y(), desired_state.get_linear_velocity().z();

  controller_->Update(stateVel, commandVel);
  Vec force = controller_->control_output();

  // Find angular displacement from desired velocity
  auto angular_displacement = std::chrono::seconds(1) * state_representation::CartesianTwist(desired_state);

  // Feed-forward angular displacement and damp the angular velocity error
  Eigen::Vector3d torque = ak_ * angular_displacement.get_orientation().vec() - ad_ * (feedback_state.get_angular_velocity() - desired_state.get_angular_velocity());

  command.set_force({force[0], force[1], force[2]});
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
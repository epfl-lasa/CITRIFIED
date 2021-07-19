#include "controllers/FrankaController.h"

#include <utility>

#include "franka_lwi/franka_lwi_utils.h"

namespace controllers {
FrankaController::FrankaController(network::InterfaceType id,
                                   const std::string& robot_name,
                                   const std::string& end_effector_name) :
    franka_(id), robot_state_(end_effector_name, robot_name), jacobian_(robot_name, 7, end_effector_name, robot_name) {
  callback_ = &FrankaController::zero_callback;
}

void FrankaController::start() {
  if (!running_ && !control_thread_.joinable()) {
    running_ = true;
    control_thread_ = std::thread([this] { control_loop(); });
  } else if (paused_) {
    paused_ = false;
  }
}

void FrankaController::pause() {
  paused_ = true;
}

void FrankaController::stop() {
  running_ = false;
  control_thread_.join();
}

state_representation::JointTorques FrankaController::zero_callback(const state_representation::CartesianState& state,
                                                                   const state_representation::Jacobian&) {
  return state_representation::JointTorques::Zero(state.get_reference_frame(), 7);
}

void FrankaController::set_callback(FrankaControllerCallback callback) {
  callback_ = std::move(callback);
}

state_representation::CartesianState FrankaController::get_state() {
  state_mutex_.lock();
  auto state_copy = robot_state_;
  state_mutex_.unlock();

  return state_copy;
}

void FrankaController::control_loop() {
  while (running_) {
    if (!paused_) {
      control_step();
    }
  }
}

void FrankaController::control_step() {
  frankalwi::proto::StateMessage<7> state_message{};
  frankalwi::proto::CommandMessage<7> command_message{};
  command_message.controlType = frankalwi::proto::JOINT_TORQUE;
  if (franka_.receive(state_message)) {
    state_mutex_.lock();
    frankalwi::utils::toCartesianState(state_message, robot_state_);
    state_mutex_.unlock();
    frankalwi::utils::toJacobian(state_message.jacobian, jacobian_);

    state_representation::JointTorques joint_command = callback_(robot_state_, jacobian_);

    frankalwi::utils::fromJointTorque(joint_command, command_message);

    franka_.send(command_message);
  }
}
}
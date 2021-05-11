#pragma once

#include <thread>
#include <mutex>

#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/Jacobian.hpp>

#include "network/interfaces.h"

namespace controllers {
typedef std::function<state_representation::JointTorques(const state_representation::CartesianState&,
                                                         const state_representation::Jacobian&)> FrankaControllerCallback;

class FrankaController {
public:
  explicit FrankaController(network::InterfaceType id,
                            const std::string& robot_name = "franka",
                            const std::string& end_effector_name = "ee");

  void start();
  void pause();
  void stop();

  void set_callback(FrankaControllerCallback callback);
  state_representation::CartesianState get_state();

private:
  void control_loop();
  void control_step();

  network::Interface franka_;
  state_representation::CartesianState robot_state_;
  state_representation::Jacobian jacobian_;
  FrankaControllerCallback callback_;

  std::thread control_thread_;
  std::mutex state_mutex_;
  bool running_ = false;
  bool paused_ = false;
};
}
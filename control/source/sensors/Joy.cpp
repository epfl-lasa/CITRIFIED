#include "sensors/Joy.h"

namespace sensors {

void Joy::start() {
  if (!keepAlive_) {
    keepAlive_ = true;
    joyThread_ = std::thread([this] { run(); });
  }
}

void Joy::stop() {
  keepAlive_ = false;
  joyThread_.join();
}

void Joy::run() {
  while (keepAlive_) {
    pure_joy::proto::JoyMessage joyCommand{};
    interface_.receive(joyCommand);
    joyMutex_.lock();
    axes_ = Eigen::VectorXd::Map(joyCommand.axes.data.data(), joyCommand.axes.data.size());
    buttons_ = Eigen::VectorXi::Map(joyCommand.buttons.data.data(), joyCommand.buttons.data.size());
    joyMutex_.unlock();
    usleep(1000);
  }
}

state_representation::CartesianPose Joy::getJoyUpdate(double gain) {
  joyMutex_.lock();
  auto axesCopy = axes_;
  joyMutex_.unlock();
  state_representation::CartesianPose pose("joy", "ee");
  pose.set_position(axesCopy(0), axesCopy(1), (axesCopy(2)-axesCopy(5)) / 2.0);
  pose = gain * pose;
  return pose;
}

}
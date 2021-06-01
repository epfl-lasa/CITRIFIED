#include "sensors/Joy.h"

namespace sensors {

Joy::Joy(const double& positionGain, const double& orientationGain) :
    positionGain_(positionGain), orientationGain_(orientationGain) {}

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
    zmq_joy::proto::JoyMessage joyCommand{};
    if (interface_.receive(joyCommand)) {
      if (!joyReady_) {
        joyReady_ = true;
      }
      joyMutex_.lock();
      axes_ = Eigen::VectorXd::Map(joyCommand.axes.data.data(), joyCommand.axes.data.size());
      buttons_ = Eigen::VectorXi::Map(joyCommand.buttons.data.data(), joyCommand.buttons.data.size());
      joyMutex_.unlock();
      usleep(1000);
    }
  }
}

void Joy::getJoyUpdate(state_representation::CartesianPose& pose) {
  if (joyReady_) {
    joyMutex_.lock();
    auto axesCopy = axes_;
    auto buttonsCopy = buttons_;
    joyMutex_.unlock();
    pose.set_position(-axesCopy(0), axesCopy(1), (axesCopy(2) - axesCopy(5)) / 2.0);
    pose.set_position(positionGain_ * pose.get_position());
    auto rot = Eigen::AngleAxisd(orientationGain_ * (buttonsCopy(5) - buttonsCopy(4)), Eigen::Vector3d::UnitZ());
    rot = rot * Eigen::AngleAxisd(orientationGain_ * axesCopy(4), Eigen::Vector3d::UnitY());
    rot = rot * Eigen::AngleAxisd(orientationGain_ * axesCopy(3), Eigen::Vector3d::UnitX());
    pose.set_orientation(Eigen::Quaterniond(rot));
  }
}

}
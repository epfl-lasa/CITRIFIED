#pragma once

#include <mutex>
#include <thread>
#include <iostream>
#include <unistd.h>

#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <network/zmq_joy_protocol.h>

#include "network/interfaces.h"

namespace sensors {

class Joy {
public:
  explicit Joy(const double& positionGain = 0.0001, const double& orientationGain = 0.0001);

  ~Joy() = default;

  void start();
  void stop();
  void run();

  void getJoyUpdate(state_representation::CartesianPose& pose);

private:
  network::Interface interface_ = network::Interface(network::InterfaceType::JOY);
  Eigen::VectorXd axes_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXi buttons_ = Eigen::VectorXi::Zero(17);

  double positionGain_;
  double orientationGain_;

  std::thread joyThread_;
  std::mutex joyMutex_;
  bool keepAlive_ = false;
  bool joyReady_ = false;
};
}
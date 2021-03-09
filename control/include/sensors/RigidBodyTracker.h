#pragma once

#include <thread>
#include <mutex>

#include <state_representation/Space/Cartesian/CartesianState.hpp>

#include "network/interfaces.h"

namespace sensors {
class RigidBodyTracker {
public:
  RigidBodyTracker(int rigidBodyID);

  void start();
  void stop();

  bool getState(StateRepresentation::CartesianState& state);
private:
  void pollThread();
  std::thread pollThread_;
  std::mutex stateMutex_;
  bool keepAlive_ = false;

  StateRepresentation::CartesianState stateEstimate_;
  network::Interface interface_;

  int rigidBodyID_;
  bool receivedState_ = false;
};
}
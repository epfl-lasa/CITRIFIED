#pragma once

#include <thread>
#include <mutex>

#include <zmq.hpp>

#include "state_representation/Space/Cartesian/CartesianState.hpp"

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
  zmq::context_t context_;
  zmq::socket_t subscriber_;

  int rigidBodyID_;
  bool receivedState_ = false;
};
}
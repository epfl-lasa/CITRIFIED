#pragma once

#include <thread>
#include <mutex>
#include <map>

#include <state_representation/space/cartesian/CartesianState.hpp>

#include "sensors/optitrack/optitrack_zmq_proto.h"
#include "network/interfaces.h"

namespace sensors {
class RigidBodyTracker {
public:
  explicit RigidBodyTracker();

  void start();
  void stop();

  bool getState(state_representation::CartesianState& state, int ID);
private:
  void pollThread();
  std::thread pollThread_;
  std::mutex stateMutex_;
  bool keepAlive_ = false;

  std::map<int, optitrack::proto::RigidBody> bodies_;
  network::Interface interface_;

};
}
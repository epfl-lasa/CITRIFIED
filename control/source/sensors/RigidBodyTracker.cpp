#include "sensors/RigidBodyTracker.h"

#include <thread>

namespace sensors {

RigidBodyTracker::RigidBodyTracker() :
    interface_(network::InterfaceType::OPTITRACK) {
  bodies_.clear();
}

void RigidBodyTracker::start() {
  // start polling thread
  keepAlive_ = true;
  pollThread_ = std::thread([this] {pollThread();});
}

void RigidBodyTracker::stop() {
  // stop polling thread
  keepAlive_ = false;
  bodies_.clear();
}

bool RigidBodyTracker::getState(state_representation::CartesianState& state, int ID) {
  bool exists = static_cast<bool>(bodies_.count(ID));
  if (exists) {
    stateMutex_.lock();
    auto rb = bodies_.at(ID);
    stateMutex_.unlock();
    state.set_position(Eigen::Vector3d(rb.x, rb.y, rb.z));
    state.set_orientation(Eigen::Quaterniond(rb.qw, rb.qx, rb.qy, rb.qz));
  }
  return exists;
}

void RigidBodyTracker::pollThread() {
  optitrack::proto::RigidBody rb{};
  while(keepAlive_) {
    if (interface_.receive(rb)) {
      stateMutex_.lock();
      bodies_.insert_or_assign(rb.id, rb);
      stateMutex_.unlock();
    }
  }
}

}
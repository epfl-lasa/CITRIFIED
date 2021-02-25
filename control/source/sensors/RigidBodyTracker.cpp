#include "sensors/RigidBodyTracker.h"

#include <thread>
#include <unistd.h>

#include "sensors/optitrack/optitrack_zmq_utils.h"

namespace sensors {

RigidBodyTracker::RigidBodyTracker(int rigidBodyID) :
  rigidBodyID_(rigidBodyID),
  stateEstimate_("rigid_body_" + std::to_string(rigidBodyID), "optitrack") {
  optitrack::utils::configureSockets(context_, subscriber_);
}

void RigidBodyTracker::start() {
  // start polling thread
  keepAlive_ = true;
  pollThread_ = std::thread([this] {pollThread();});
}

void RigidBodyTracker::stop() {
  // stop polling thread
  keepAlive_ = false;
  receivedState_ = false;
}

bool RigidBodyTracker::getState(StateRepresentation::CartesianState& state) {
  if (receivedState_) {
    stateMutex_.lock();
    state = stateEstimate_;
    stateMutex_.unlock();
  }
  return receivedState_;
}

void RigidBodyTracker::pollThread() {
  optitrack::proto::RigidBody rb{};
  while(keepAlive_) {
    if (optitrack::utils::poll(subscriber_, rb)) {
      if (rb.id == rigidBodyID_) {
        receivedState_ = true;
        stateMutex_.lock();
        stateEstimate_.set_position(Eigen::Vector3d(rb.x, rb.y, rb.z));
        stateEstimate_.set_orientation(Eigen::Quaterniond(rb.qw, rb.qx, rb.qy, rb.qz));
        stateMutex_.unlock();
      }
    }
    usleep(100);
  }
}

}
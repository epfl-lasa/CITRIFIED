#pragma once

#include <ros/ros.h>
#include <zmq.hpp>

#include "franka_lwi_communication_protocol.h"

namespace zmq_bridge {

class ZMQBridge {
public:
  ZMQBridge(ros::NodeHandle* nh, const std::string& uri);

  void run();

private:
  bool poll(frankalwi::proto::StateMessage<7>& msg);
  void publishState(const frankalwi::proto::StateMessage<7>& msg);
  void publishAttractor(const frankalwi::proto::StateMessage<7>& msg);

  ros::Publisher jointStatePub_;
  ros::Publisher attractorPub_;

  zmq::context_t context_;
  zmq::socket_t sub_;

  frankalwi::proto::StateMessage<7> zmqMsg_;
};
}
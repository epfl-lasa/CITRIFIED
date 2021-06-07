#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <zmq.hpp>

#include "zmq_joy_protocol.h"

namespace zmq_joy {

class PureZMQJoy {
public:
  PureZMQJoy(ros::NodeHandle nh);

  void run();

private:
  ros::Subscriber sub_;

  zmq::context_t context_;
  zmq::socket_t pub_;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& rosMsg);
  bool send(const zmq_joy::proto::JoyMessage& msg);
};
}
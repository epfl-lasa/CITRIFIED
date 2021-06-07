#include <ros/ros.h>
#include <zmq.hpp>

#include "zmq_bridge/ZMQBridge.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "zmq_bridge");
  ros::NodeHandle n;
  zmq_bridge::ZMQBridge bridge(&n, "0.0.0.0:9999");
  bridge.run();

  return 0;
}
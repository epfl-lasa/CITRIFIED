#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <zmq.hpp>

#include "pure_zmq_joy/PureZMQJoy.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_republisher");
  ros::NodeHandle n;
  zmq_joy::PureZMQJoy joy(n);
  joy.run();

  return 0;
}
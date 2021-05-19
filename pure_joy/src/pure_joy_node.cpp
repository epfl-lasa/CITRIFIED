#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <zmq.hpp>

#include "pure_joy/PureJoy.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_republisher");
  ros::NodeHandle n;
  pure_joy::PureJoy joy(n);
  joy.run();

  return 0;
}
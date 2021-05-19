#include "pure_joy/PureJoy.h"

namespace pure_joy {

PureJoy::PureJoy(ros::NodeHandle nh) {
  sub_ = nh.subscribe("/joy", 10, &PureJoy::joyCallback, this);

  context_ = zmq::context_t(1);
  pub_ = zmq::socket_t(context_, ZMQ_PUB);
  pub_.connect("tcp://0.0.0.0:8888");
}

void PureJoy::run() {
  while (ros::ok()) {
    ros::spinOnce();
  }
}

void PureJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& rosMsg) {
  ROS_ERROR_STREAM(*rosMsg);
  proto::JoyMessage zmqMsg;
  for (std::size_t button = 0; button < 17; ++button) {
    zmqMsg.buttons[button] = rosMsg->buttons[button];
  }
  for (std::size_t ax = 0; ax < 6; ++ax) {
    zmqMsg.axes[ax] = rosMsg->axes[ax];
  }
  send(zmqMsg);
}

bool PureJoy::send(const pure_joy::proto::JoyMessage& msg) {
  zmq::message_t message(sizeof(msg));
  memcpy(message.data(), &msg, sizeof(msg));
  auto res = pub_.send(message, zmq::send_flags::none);
  return res.has_value();
}
}
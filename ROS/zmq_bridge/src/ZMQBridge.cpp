#include "zmq_bridge/ZMQBridge.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

namespace zmq_bridge {

ZMQBridge::ZMQBridge(ros::NodeHandle* nh, const std::string& uri) {
  jointStatePub_ = nh->advertise<sensor_msgs::JointState>("/joint_states", 10);
  attractorPub_ = nh->advertise<geometry_msgs::PoseStamped>("/attractor", 10);

  context_ = zmq::context_t(1);
  sub_ = zmq::socket_t(context_, ZMQ_SUB);
  sub_.set(zmq::sockopt::conflate, 1);
  sub_.set(zmq::sockopt::subscribe, "");
  sub_.connect("tcp://" + uri);
}

void ZMQBridge::run() {
  while (ros::ok()) {
    if (poll(zmqMsg_)) {
      publishState(zmqMsg_);
      publishAttractor(zmqMsg_);
    }
    ros::spinOnce();
  }
}

bool ZMQBridge::poll(frankalwi::proto::StateMessage<7>& msg) {
  zmq::message_t message;
  auto res = sub_.recv(message, zmq::recv_flags::dontwait);
  if (res) {
    msg = *message.data<frankalwi::proto::StateMessage<7>>();
  }
  return res.has_value();
}

void ZMQBridge::publishState(const frankalwi::proto::StateMessage<7>& msg) {
  auto jointState = sensor_msgs::JointState();
  jointState.header.stamp = ros::Time::now();
  jointState.name = std::vector<std::string>{
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
  };
  jointState.position = std::vector<double>{msg.jointPosition.data.begin(), msg.jointPosition.data.end()};
  jointState.velocity = std::vector<double>{msg.jointVelocity.data.begin(), msg.jointVelocity.data.end()};
  jointState.effort = std::vector<double>{msg.jointTorque.data.begin(), msg.jointTorque.data.end()};
  jointStatePub_.publish(jointState);
}

void ZMQBridge::publishAttractor(const frankalwi::proto::StateMessage<7>& msg) {
  auto pose = geometry_msgs::PoseStamped();
  pose.header.frame_id = "panda_link0";
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = msg.eePose.position.x;
  pose.pose.position.y = msg.eePose.position.y;
  pose.pose.position.z = msg.eePose.position.z;
  pose.pose.orientation.w = msg.eePose.orientation.w;
  pose.pose.orientation.x = msg.eePose.orientation.x;
  pose.pose.orientation.y = msg.eePose.orientation.y;
  pose.pose.orientation.z = msg.eePose.orientation.z;
  attractorPub_.publish(pose);
}
}
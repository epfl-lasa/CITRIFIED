#pragma once

#include <zmq.hpp>

#include "sensors/optitrack/optitrack_zmq_proto.h"

namespace optitrack::utils {

void configureSockets(zmq::context_t& context, zmq::socket_t& subscriber,
                      const std::string& optitrack_uri = "0.0.0.0:5511") {
  context = zmq::context_t(1);

  subscriber = zmq::socket_t(context, ZMQ_SUB);
  subscriber.set(zmq::sockopt::conflate, 1);
  subscriber.set(zmq::sockopt::subscribe, "");
  subscriber.bind("tcp://" + optitrack_uri);
}

bool poll(zmq::socket_t& subscriber, optitrack::proto::RigidBody& rb) {
  zmq::message_t message;
  auto res = subscriber.recv(message, zmq::recv_flags::dontwait);
  if (res) {
    rb = *message.data<optitrack::proto::RigidBody>();
  }
  return res.has_value();
}

}
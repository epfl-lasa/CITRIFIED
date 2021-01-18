
#pragma once

#include <zmq.hpp>

namespace network {

void configure(zmq::context_t& context, zmq::socket_t& publisher, zmq::socket_t& subscriber,
               const std::string& state_uri="0.0.0.0:5550", const std::string& command_uri="0.0.0.0:5551") {
  context = zmq::context_t(1);

  subscriber = zmq::socket_t(context, ZMQ_SUB);
  subscriber.set(zmq::sockopt::conflate, 1);
  subscriber.set(zmq::sockopt::subscribe, "");
  subscriber.bind("tcp://" + state_uri);

  publisher = zmq::socket_t(context, ZMQ_PUB);
  publisher.bind("tcp://" + command_uri);
}

}
#pragma once

#include <zmq.hpp>

namespace network::zmq_interface {

void configureSockets(zmq::context_t& context, zmq::socket_t& publisher, zmq::socket_t& subscriber,
                      const std::string& state_uri = "0.0.0.0:5550", const std::string& command_uri = "0.0.0.0:5551",
                      const bool bind = true) {
  context = zmq::context_t(1);

  subscriber = zmq::socket_t(context, ZMQ_SUB);
  subscriber.set(zmq::sockopt::conflate, 1);
  subscriber.set(zmq::sockopt::subscribe, "");

  publisher = zmq::socket_t(context, ZMQ_PUB);

  if (bind) {
    subscriber.bind("tcp://" + state_uri);
    publisher.bind("tcp://" + command_uri);
  } else {
    subscriber.connect("tcp://" + state_uri);
    publisher.connect("tcp://" + command_uri);
  }
}

template <typename T> bool send(zmq::socket_t& publisher, const T& obj) {
  zmq::message_t message(sizeof(obj));
  memcpy(message.data(), &obj, sizeof(obj));
  auto res = publisher.send(message, zmq::send_flags::none);

  return res.has_value();
}

template <typename T> bool receive(zmq::socket_t& subscriber, T& obj,
                                   const zmq::recv_flags flags = zmq::recv_flags::none) {
  zmq::message_t message;
  auto res = subscriber.recv(message, flags);
  if (res) {
    obj = *message.data<T>();
  }
  return res.has_value();
}

template <typename T> bool poll(zmq::socket_t& subscriber, T& obj) {
  return receive(subscriber, obj, zmq::recv_flags::dontwait);
}

}

#pragma once

#include <zmq.hpp>

namespace network::zmq_interface {

inline void configurePubSubSockets(zmq::context_t& context,
                                   zmq::socket_t& publisher,
                                   zmq::socket_t& subscriber,
                                   const std::string& subscriber_uri,
                                   const std::string& publisher_uri,
                                   const bool bind = true) {
  context = zmq::context_t(1);

  subscriber = zmq::socket_t(context, ZMQ_SUB);
  subscriber.set(zmq::sockopt::conflate, 1);
  subscriber.set(zmq::sockopt::subscribe, "");

  publisher = zmq::socket_t(context, ZMQ_PUB);

  if (bind) {
    subscriber.bind("tcp://" + subscriber_uri);
    publisher.bind("tcp://" + publisher_uri);
  } else {
    subscriber.connect("tcp://" + subscriber_uri);
    publisher.connect("tcp://" + publisher_uri);
  }
}

inline void configurePairSocket(zmq::context_t& context,
                                zmq::socket_t& socket,
                                const std::string& socket_uri,
                                const bool bind = true) {
  context = zmq::context_t(1);

  socket = zmq::socket_t(context, ZMQ_PAIR);

  if (bind) {
    socket.bind("tcp://" + socket_uri);
  } else {
    socket.connect("tcp://" + socket_uri);
  }
}

template<typename T>
inline bool send(zmq::socket_t& socket, const T& obj) {
  zmq::message_t message(sizeof(obj));
  memcpy(message.data(), &obj, sizeof(obj));
  auto res = socket.send(message, zmq::send_flags::none);
  return res.has_value();
}

inline bool send_string(zmq::socket_t& socket, const std::string& obj) {
  zmq::message_t message(obj.size());
  memcpy(message.data(), obj.data(), obj.size());
  auto res = socket.send(message, zmq::send_flags::none);
  return res.has_value();
}

template<typename T>
inline bool receive(zmq::socket_t& socket, T& obj, const zmq::recv_flags flags = zmq::recv_flags::none) {
  zmq::message_t message;
  auto res = socket.recv(message, flags);
  if (res) {
    obj = *message.data<T>();
  }
  return res.has_value();
}

template<typename T>
inline bool poll(zmq::socket_t& socket, T& obj) {
  return receive(socket, obj, zmq::recv_flags::dontwait);
}

}

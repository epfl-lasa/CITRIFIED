#pragma once

#include <zmq.hpp>
#include "network/zmq_interface.h"

namespace network {

enum InterfaceType {
  FRANKA_LWI, OPTITRACK, GPR
};

class Interface {
public:
  explicit Interface(InterfaceType type);

  template <typename T> bool receive(T& message);

  template <typename T> bool poll(T& message);

  template <typename T> bool send(const T& message);

private:
  zmq::context_t context_;
  zmq::socket_t publisher_;
  zmq::socket_t subscriber_;

};

inline Interface::Interface(InterfaceType type) {
  std::string subscriber_uri, publisher_uri;
  bool bind;
  switch (type) {
    case FRANKA_LWI:
      bind = true;
      subscriber_uri = "0.0.0.0:5550";
      publisher_uri = "0.0.0.0:5551";
      break;
    case OPTITRACK:
      bind = true;
      subscriber_uri = "0.0.0.0:5511";
      publisher_uri = "0.0.0.0:5512";
      break;
    case GPR:
      bind = false;
      subscriber_uri = "0.0.0.0:7770";
      publisher_uri = "0.0.0.0:7771";
      break;
  }
  zmq_interface::configureSockets(context_, publisher_, subscriber_, subscriber_uri, publisher_uri, bind);
}

template <typename T> bool Interface::receive(T& message) {
  if (subscriber_.connected()) {
    return zmq_interface::receive(subscriber_, message);
  }
  return false;
}

template <typename T> bool Interface::poll(T& message) {
  if (subscriber_.connected()) {
    return zmq_interface::poll(subscriber_, message);
  }
  return false;
}

template <typename T> bool Interface::send(const T& message) {
  if (publisher_.connected()) {
    return zmq_interface::send(publisher_, message);
  }
  return false;
}

}
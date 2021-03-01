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

Interface::Interface(InterfaceType type) {
  std::string subscriber_uri, publisher_uri;
  bool bind;
  switch (type) {
    case FRANKA_LWI:
      bind = true;
      subscriber_uri = "0.0.0.0:5550";
      publisher_uri = "0.0.0.0:5551";
      break;
    case OPTITRACK:
      bind = false;
      subscriber_uri = "0.0.0.0:6660";
      publisher_uri = "0.0.0.0:6661";
      break;
    case GPR:
      bind = false;
      subscriber_uri = "0.0.0.0:7770";
      publisher_uri = "0.0.0.0:7771";
      break;
  }
  network::zmq_interface::configureSockets(context_, publisher_, subscriber_, subscriber_uri, publisher_uri, bind);
}

template <typename T> bool Interface::receive(T& message) {
  if (!subscriber_.connected()) { return false; }
  return network::zmq_interface::receive(subscriber_, message);
}

template <typename T> bool Interface::poll(T& message) {
  if (!subscriber_.connected()) { return false; }
  return network::zmq_interface::poll(subscriber_, message);
}

template <typename T> bool Interface::send(const T& message) {
  if (!publisher_.connected()) { return false; }
  return network::zmq_interface::send(publisher_, message);
}

}

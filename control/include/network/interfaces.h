#pragma once

#include <zmq.hpp>
#include "network/zmq_interface.h"

namespace network {

enum InterfaceType {
  FRANKA_PAPA_16, FRANKA_QUEBEC_17, OPTITRACK, GPR, JOY
};

class Interface {
public:
  explicit Interface(InterfaceType type);

  template<typename T>
  bool receive(T& message);

  template<typename T>
  bool poll(T& message);

  template<typename T>
  bool send(const T& message);

  bool send_string(const std::string& message);

private:
  InterfaceType type_;
  zmq::context_t context_;
  zmq::socket_t publisher_;
  zmq::socket_t subscriber_;

};

inline Interface::Interface(InterfaceType type) : type_(type) {
  switch (type_) {
    case FRANKA_PAPA_16:
      zmq_interface::configurePubSubSockets(context_, publisher_, subscriber_, "0.0.0.0:1601", "0.0.0.0:1602", true);
      break;
    case FRANKA_QUEBEC_17:
      zmq_interface::configurePubSubSockets(context_, publisher_, subscriber_, "0.0.0.0:1701", "0.0.0.0:1702", true);
      break;
    case OPTITRACK:
      zmq_interface::configurePubSubSockets(context_, publisher_, subscriber_, "0.0.0.0:5511", "0.0.0.0:5512", true);
      break;
    case GPR:
      zmq_interface::configurePairSocket(context_, subscriber_, "0.0.0.0:7777", true);
      break;
    case JOY:
      zmq_interface::configureSubSocket(context_, subscriber_, "0.0.0.0:8888", true);
      break;
  }
}

template<typename T>
inline bool Interface::receive(T& message) {
  if (subscriber_.connected()) {
    return zmq_interface::receive(subscriber_, message);
  }
  return false;
}

template<typename T>
inline bool Interface::poll(T& message) {
  if (subscriber_.connected()) {
    return zmq_interface::poll(subscriber_, message);
  }
  return false;
}

template<typename T>
inline bool Interface::send(const T& message) {
  switch (type_) {
    case GPR:
      if (subscriber_.connected()) {
        return zmq_interface::send(subscriber_, message);
      }
      break;
    default:
      if (publisher_.connected()) {
        return zmq_interface::send(publisher_, message);
      }
      break;
  }
  return false;
}

inline bool Interface::send_string(const std::string& message) {
  switch (type_) {
    case GPR:
      if (subscriber_.connected()) {
        return zmq_interface::send_string(subscriber_, message);
      }
      break;
    default:
      if (publisher_.connected()) {
        return zmq_interface::send_string(publisher_, message);
      }
      break;
  }
  return false;
}

}

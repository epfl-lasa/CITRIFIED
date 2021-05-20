#pragma once

#include <array>

namespace zmq_joy::proto {

struct Buttons {
  int& operator[](std::size_t i) {
    return data[i % 17];
  }
  std::array<int, 17> data;
};

struct Axes {
  double& operator[](std::size_t i) {
    return data[i % 6];
  }
  std::array<double, 6> data;
};

struct JoyMessage {
  Axes axes;
  Buttons buttons;
};
}
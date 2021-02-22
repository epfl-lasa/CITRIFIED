#pragma once

#include <chrono>
#include <array>

namespace netft_rdt_driver {

struct Vec3D {
  Vec3D() : x(0), y(0), z(0) {}
  explicit Vec3D(std::array<double, 3> vec) : x(vec[0]), y(vec[1]), z(vec[2]) {}
  double x;
  double y;
  double z;
};

struct RawWrenchMessage {
  std::chrono::system_clock::time_point time = std::chrono::system_clock::now();
  Vec3D force = Vec3D(std::array<double, 3>{0, 0, 0});
  Vec3D torque = Vec3D(std::array<double, 3>{0, 0, 0});
};

class INetFTRDTDriver {
public:

  virtual void getData(RawWrenchMessage& data) = 0;

  virtual bool waitForNewData() = 0;
};
}
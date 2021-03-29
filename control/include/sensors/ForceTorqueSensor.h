#pragma once

#include "sensors/netft_rdt_driver/INetFTRDTDriver.h"

#include <memory>
#include <state_representation/space/cartesian/CartesianWrench.hpp>

namespace sensors {

struct ToolSpec {
  Eigen::Vector3d centerOfMass;
  double mass;
};

class ForceTorqueSensor {
public:
  explicit ForceTorqueSensor(const std::string& sensorName, const std::string& address, std::size_t sensorTimeoutMs,
                             ToolSpec tool, bool mock = false);

  bool computeBias(const Eigen::Matrix3d& worldToEERotation, std::size_t numPoints);

  bool readContactWrench(state_representation::CartesianWrench& wrench, const Eigen::Matrix3d& worldToFTRotation);

  bool readRawData(state_representation::CartesianWrench& wrench);

  bool readBias(state_representation::CartesianWrench& bias);

  bool biasOK();

  void resetBias();

private:
  std::string sensorName_;
  std::unique_ptr<netft_rdt_driver::INetFTRDTDriver> netftRDTDriver_;
  ToolSpec tool_;

  state_representation::CartesianWrench bias_;
  std::size_t biasCount_ = 0;
  bool biasOk_ = false;
  Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81);
};
}
#include "sensors/ForceTorqueSensor.h"

#include <utility>

#include "sensors/netft_rdt_driver/NetFTRDTDriver.h"
#include "sensors/netft_rdt_driver/MockNetFTRDTDriver.h"

namespace sensors {

ForceTorqueSensor::ForceTorqueSensor(const std::string& sensorName, const std::string& address,
                                     std::size_t sensorTimeoutMs, ToolSpec tool, bool simulation) :
    sensorName_(sensorName),
    tool_(std::move(tool)),
    bias_(StateRepresentation::CartesianWrench(sensorName + "Bias", sensorName)) {
  if (simulation) {
    netftRDTDriver_ = std::make_unique<netft_rdt_driver::MockNetFTRDTDriver>();
  } else {
    netftRDTDriver_ = std::make_unique<netft_rdt_driver::NetFTRDTDriver>(address, sensorTimeoutMs);
  }
}

bool ForceTorqueSensor::computeBias(const Eigen::Matrix3d& worldToFTRotation, std::size_t numPoints) {
  if (!biasOk_) {
    StateRepresentation::CartesianWrench tmp(sensorName_, sensorName_);
    if (!readRawData(tmp)) {
      return false;
    }
    Eigen::Vector3d loadForceFTFrame = worldToFTRotation.transpose() * gravity_ * tool_.mass;
    tmp.set_force(tmp.get_force() - loadForceFTFrame);
    tmp.set_torque(tmp.get_torque() - loadForceFTFrame.cross(tool_.centerOfMass));
    bias_.set_wrench(bias_.get_wrench() + tmp.get_wrench());
    ++biasCount_;
    if (biasCount_ >= numPoints) {
      bias_.set_wrench(bias_.get_wrench() / double(numPoints));
      biasOk_ = true;
    }
  }
  return biasOk_;
}

bool ForceTorqueSensor::readContactWrench(StateRepresentation::CartesianWrench& wrench,
                                          const Eigen::Matrix3d& worldToFTRotation) {
  StateRepresentation::CartesianWrench tmp(sensorName_, sensorName_);
  if (!readRawData(tmp)) {
    return false;
  }
  Eigen::Vector3d loadForceFTFrame = worldToFTRotation.transpose() * gravity_ * tool_.mass;
  tmp.set_force(tmp.get_force() - loadForceFTFrame);
  tmp.set_torque(tmp.get_torque() - loadForceFTFrame.cross(tool_.centerOfMass));
  wrench.set_wrench(tmp.get_wrench() - bias_.get_wrench());
  return true;
}

bool ForceTorqueSensor::readRawData(StateRepresentation::CartesianWrench& wrench) {
  netft_rdt_driver::RawWrenchMessage rawMessage;
  if (wrench.get_reference_frame() != sensorName_) {
    wrench.set_reference_frame(sensorName_);
    std::cout << "[ForceTorqueSensor::getRawData] Warning: Resetting reference frame of contact wrench to sensor frame!"
              << std::endl;
  }
  if (netftRDTDriver_->waitForNewData()) {
    netftRDTDriver_->getData(rawMessage);
    wrench.set_force(Eigen::Vector3d(rawMessage.force.x, rawMessage.force.y, rawMessage.force.z));
    wrench.set_torque(Eigen::Vector3d(rawMessage.torque.x, rawMessage.torque.y, rawMessage.torque.z));
    return true;
  } else {
    return false;
  }
}

bool ForceTorqueSensor::readBias(StateRepresentation::CartesianWrench& wrench) {
  if (biasOk_) {
    wrench = bias_;
    return true;
  } else {
    return false;
  }
}

void ForceTorqueSensor::resetBias() {
  biasOk_ = false;
  bias_.set_zero();
  biasCount_ = 0;
}
}

#pragma once

#include <franka_lwi/franka_lwi_communication_protocol.h>
#include <passive-ds-control/passive_ds_controller.h>

namespace controller {

class CartesianLinearSpaceController {
public:
  CartesianLinearSpaceController();
  CartesianLinearSpaceController(double d0, double d1);
  void setDamping(double d0, double d1);
  frankalwi::proto::CommandMessage<7> getJointTorque(frankalwi::proto::StateMessage<7> state,
                                                     const std::vector<double>& twist);
  static frankalwi::proto::CommandMessage<7> getJointTorque(frankalwi::proto::StateMessage<7> state,
                                                            const Eigen::Matrix<double, 6, 1>& wrench);
protected:
  virtual Eigen::Matrix<double, 6, 1> getWrenchCommand(frankalwi::proto::StateMessage<7> state,
                                                       const std::vector<double>& twist);
  PassiveDSController* controller_;
  double d0_ = 50;
  double d1_ = 50;
  double maxTankLevel_ = 100;
  double dz_ = 0.01;
  double maxForce = 50;
};

class CartesianAngularSpaceController : public CartesianLinearSpaceController {
public:
  CartesianAngularSpaceController() = default;
  explicit CartesianAngularSpaceController(double k);
  CartesianAngularSpaceController(double k, double d);
  void setStiffness(double k);
  void setDamping(double d);

protected:
  double k_ = 25;
  double d_ = 0;
  double maxTorque = 50;
  Eigen::Matrix<double, 6, 1> getWrenchCommand(frankalwi::proto::StateMessage<7> state,
                                               const std::vector<double>& twist) override;
};

class CartesianPoseController {
public:
  CartesianPoseController(double linearD0, double linearD1, double angularK);

  CartesianLinearSpaceController linearController;
  CartesianAngularSpaceController angularController;
  frankalwi::proto::CommandMessage<7> getJointTorque(frankalwi::proto::StateMessage<7> state,
                                                     const std::vector<double>& twist);
};

}
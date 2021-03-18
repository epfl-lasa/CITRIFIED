
#pragma once

#include <franka_lwi/franka_lwi_communication_protocol.h>
#include <passive-ds-control/passive_ds_controller.h>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>
#include <state_representation/robot/JointTorques.hpp>
#include <state_representation/robot/Jacobian.hpp>

namespace controller {

class CartesianLinearSpaceController {
public:
  explicit CartesianLinearSpaceController(double d0 = 50, double d1 = 50);
  void setDamping(double d0, double d1);

  frankalwi::proto::CommandMessage<7> getJointTorque(frankalwi::proto::StateMessage<7> state,
                                                     const std::vector<double>& twist);

  static frankalwi::proto::CommandMessage<7> getJointTorque(frankalwi::proto::StateMessage<7> state,
                                                            const Eigen::Matrix<double, 6, 1>& wrench);

  virtual state_representation::CartesianWrench getWrenchCommand(const state_representation::CartesianTwist& state,
                                                                 const state_representation::CartesianTwist& command);
protected:
  virtual Eigen::Matrix<double, 6, 1> getWrenchCommand(frankalwi::proto::StateMessage<7> state,
                                                       const std::vector<double>& twist);

  PassiveDSController* controller_;
  double d0_;
  double d1_;
  double maxTankLevel_ = 100;
  double dz_ = 0.01;
  double maxForce = 50;
};

class CartesianAngularSpaceController : public CartesianLinearSpaceController {
public:
  explicit CartesianAngularSpaceController(double k = 1, double d = 1);
  void setStiffness(double k);
  void setDamping(double d);

  state_representation::CartesianWrench getWrenchCommand(const state_representation::CartesianTwist& state,
                                                         const state_representation::CartesianTwist& command) override;
protected:
  double k_;
  double d_;
  double maxTorque = 50;
  Eigen::Matrix<double, 6, 1> getWrenchCommand(frankalwi::proto::StateMessage<7> state,
                                               const std::vector<double>& twist) override;
};

class CartesianPoseController {
public:
  explicit CartesianPoseController(double linearD0 = 1, double linearD1 = 1, double angularK = 1, double angularD = 0);

  CartesianLinearSpaceController linearController;
  CartesianAngularSpaceController angularController;
  frankalwi::proto::CommandMessage<7> getJointTorque(frankalwi::proto::StateMessage<7> state,
                                                     const std::vector<double>& twist);

  state_representation::JointTorques getJointTorque(const state_representation::CartesianTwist& state,
                                                    const state_representation::CartesianTwist& command,
                                                    const state_representation::Jacobian& jacobian);
};

}
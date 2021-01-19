
#include "controllers/CartesianPoseController.h"

#include <iostream>

namespace controller {

CartesianLinearSpaceController::CartesianLinearSpaceController() {
  controller_ = new PassiveDSController(3, d0_, d1_, maxTankLevel_, dz_);
}

CartesianLinearSpaceController::CartesianLinearSpaceController(double d0, double d1) : d0_(d0), d1_(d1) {
  controller_ = new PassiveDSController(3, d0_, d1_, maxTankLevel_, dz_);
}

void CartesianLinearSpaceController::setDamping(double d0, double d1) {
  d0_ = d0;
  d1_ = d1;
  controller_->set_damping_eigval(static_cast<realtype>(d0_), static_cast<realtype>(d1_));
}

Eigen::Matrix<double, 6, 1> CartesianLinearSpaceController::getWrenchCommand(frankalwi::proto::StateMessage<7> state,
                                                                             const std::vector<double>& twist) {
  Vec velocity(3);
  velocity[0] = state.eeTwist.linear.x;
  velocity[1] = state.eeTwist.linear.y;
  velocity[2] = state.eeTwist.linear.z;

  Vec desiredVelocity(3);
  desiredVelocity[0] = twist.at(0);
  desiredVelocity[1] = twist.at(1);
  desiredVelocity[2] = twist.at(2);

  controller_->Update(velocity, desiredVelocity);
  Vec force = controller_->control_output();

  if (force.norm() > maxForce) {
    force = force.normalized() * maxForce;
  }

  Eigen::Matrix<double, 6, 1> wrench;
  wrench << force[0], force[1], force[2], 0, 0, 0;
  return wrench;
}

frankalwi::proto::CommandMessage<7> CartesianLinearSpaceController::getJointTorque(frankalwi::proto::StateMessage<7> state,
                                                                                   const std::vector<double>& twist) {
  auto wrench = getWrenchCommand(state, twist);
  return getJointTorque(state, wrench);
}

frankalwi::proto::CommandMessage<7> CartesianLinearSpaceController::getJointTorque(frankalwi::proto::StateMessage<7> state,
                                                                                   const Eigen::Matrix<double, 6, 1>& wrench) {
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(state.jacobian.data());
  Eigen::Matrix<double, 7, 1> torques = jacobian.transpose() * wrench;

  frankalwi::proto::CommandMessage<7> command{};
  Eigen::MatrixXd::Map(command.jointTorque.data.data(), 7, 1) = torques.array();
  return command;
}
CartesianAngularSpaceController::CartesianAngularSpaceController(double k) : k_(k) {
}

CartesianAngularSpaceController::CartesianAngularSpaceController(double k, double d) : k_(k),
                                                                                       d_(d) {
}

void CartesianAngularSpaceController::setStiffness(double k) {
  k_ = k;
}
void CartesianAngularSpaceController::setDamping(double d) {
  d_ = d;
}

Eigen::Matrix<double, 6, 1> CartesianAngularSpaceController::getWrenchCommand(frankalwi::proto::StateMessage<7> state,
                                                                              const std::vector<double>& twist) {
  Eigen::Vector3d velocity(3);
  velocity[0] = state.eeTwist.angular.x;
  velocity[1] = state.eeTwist.angular.y;
  velocity[2] = state.eeTwist.angular.z;

  Eigen::Vector3d desiredVelocity(3);
  if (twist.size() == 6) {
    desiredVelocity[0] = twist.at(3);
    desiredVelocity[1] = twist.at(4);
    desiredVelocity[2] = twist.at(5);
  } else {
    desiredVelocity[0] = twist.at(0);
    desiredVelocity[1] = twist.at(1);
    desiredVelocity[2] = twist.at(2);
  }

  Eigen::Vector3d torque = k_ * desiredVelocity - d_ * (velocity - desiredVelocity);

  if (torque.norm() > maxTorque) {
    torque = torque.normalized() * maxTorque;
  }

  Eigen::Matrix<double, 6, 1> wrench;
  wrench << 0, 0, 0, torque[0], torque[1], torque[2];
  return wrench;
}

CartesianPoseController::CartesianPoseController(double linearD0,
                                                 double linearD1,
                                                 double angularK) : linearController(linearD0, linearD1),
                                                                    angularController(angularK) {

}

frankalwi::proto::CommandMessage<7> CartesianPoseController::getJointTorque(frankalwi::proto::StateMessage<7> state,
                                                                            const std::vector<double>& twist) {
  auto linearCommand = linearController.getJointTorque(state, twist);
  auto angularCommand = angularController.getJointTorque(state, twist);

  auto linearTorque = Eigen::Matrix<double, 7, 1>(linearCommand.jointTorque.data.data());
  auto angularTorque = Eigen::Matrix<double, 7, 1>(angularCommand.jointTorque.data.data());

  // optional: arbitrate between linear and angular commands depending on priority
  auto commandTorque = linearTorque + angularTorque;

  // optional: treat wrench as desired acceleration, and convert to actual torque
  //  auto massMatrix = Eigen::Map<const Eigen::Matrix<double, 7, 7> >(state.mass.data());
  //  Eigen::Matrix<double, 7, 1> jointTorque = massMatrix * commandTorque;

  Eigen::Matrix<double, 7, 1> jointTorque = commandTorque;

  frankalwi::proto::CommandMessage<7> command{};
  Eigen::MatrixXd::Map(command.jointTorque.data.data(), 7, 1) = jointTorque.array();

  return command;
}
}
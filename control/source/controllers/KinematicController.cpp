#include "controllers/KinematicController.h"

#include <iostream>

namespace controller {

KinematicController::KinematicController() : robot_(FRANKA_PANDA) {

}

frankalwi::proto::CommandMessage<7> KinematicController::getJointTorque(frankalwi::proto::StateMessage<7> state,
                                                                        const std::vector<double>& twist) {
  KDL::JntArray q(robot_.dof), qdot(robot_.dof), qdotDesired(robot_.dof);
  KDL::Twist v;

  q.data = Eigen::Map<Eigen::VectorXd>(state.jointPosition.data.data(), 7);
  qdot.data = Eigen::Map<Eigen::VectorXd>(state.jointVelocity.data.data(), 7);
  v.vel = KDL::Vector(twist.at(0), twist.at(1), twist.at(2));
  v.rot = KDL::Vector(twist.at(3), twist.at(4), twist.at(5));

  frankalwi::proto::CommandMessage<7> command{};
  if (robot_.ikVel(q, v, qdotDesired) < 0) {
    std::cerr << "IK Velocity Error" << std::endl;
    return command;
  }

  Eigen::VectorXd torque = gain * (qdotDesired.data - qdot.data);
  Eigen::MatrixXd::Map(command.jointTorque.data.data(), 7, 1) = torque.array();

  return command;
}

}
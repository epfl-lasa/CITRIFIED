#include <state_representation/space/cartesian/CartesianState.hpp>
#include <dynamical_systems/Linear.hpp>
#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "network/interfaces.h"
#include "controllers/impedance/CartesianTwistController.hpp"
#include "franka_lwi/franka_lwi_utils.h"
#include "sensors/RigidBodyTracker.h"

int main(int, char**) {
  sensors::RigidBodyTracker tracker;
  tracker.start();
  int robotBaseID = 1;  // the OptiTrack streaming ID for the robot base frame
  int attractorID = 2;  // the OptiTrack streaming ID for the attractor frame
  Eigen::Vector3d attractor_offset(0, 0, 0.2);  // the distance offset to follow (in attractor frame)
  Eigen::Quaterniond attractor_rotation(0, 0, 1, 0);  // the rotation offset to follow (in attractor frame)

  state_representation::CartesianPose offset("offset", attractor_offset, attractor_rotation, "attractor");
  state_representation::CartesianState attractor("attractor", "optitrack");
  state_representation::CartesianPose robotInOptitrack("robot_base", "optitrack");
  state_representation::CartesianState robot_ee("robot_ee", "robot_base");
  state_representation::Jacobian jacobian("franka", 7, "robot_ee", "robot_base");

  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  dynamical_systems::Linear<state_representation::CartesianState>
      DS(state_representation::CartesianState::Identity("attractor", "robot_base"), gains);

  controllers::impedance::CartesianTwistController ctrl(100, 100, 5, 5);

  network::Interface franka(network::InterfaceType::FRANKA_PAPA_16);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};
  command.controlType = frankalwi::proto::JOINT_TORQUE;

  std::cout << "Waiting for optitrack data for attractor and robot base..." << std::endl;
  while (!tracker.getState(robotInOptitrack, robotBaseID) || !tracker.getState(attractor, attractorID)) {}
  std::cout << "OptiTrack ready!" << std::endl;

  while (franka.receive(state)) {
    frankalwi::utils::toCartesianState(state, robot_ee);
    frankalwi::utils::toJacobian(state.jacobian, jacobian);

    tracker.getState(robotInOptitrack, robotBaseID);
    tracker.getState(attractor, attractorID);

    auto attractorInRobot = robotInOptitrack.inverse() * attractor;
    DS.set_attractor(attractorInRobot * offset);

    state_representation::CartesianTwist twist = DS.evaluate(robot_ee);
    twist.clamp(0.5, 1.0);

    state_representation::JointTorques joint_command = ctrl.compute_command(twist, robot_ee, jacobian);

    frankalwi::utils::fromJointTorque(joint_command, command);
    franka.send(command);
  }
}
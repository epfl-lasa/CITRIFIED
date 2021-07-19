#include <vector>
#include <iostream>

#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <dynamical_systems/Linear.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "controllers/impedance/CartesianTwistController.hpp"
#include "franka_lwi/franka_lwi_utils.h"
#include "network/interfaces.h"
#include "sensors/Joy.h"

void throttledPrint(const state_representation::CartesianState& robot,
                    const state_representation::CartesianPose& attractor,
                    const state_representation::CartesianTwist& commandTwist,
                    const state_representation::JointTorques& commandTorque,
                    int skip) {
  static int count = 0;
  if (count > skip) {

    std::cout << "====================" << std::endl;
    std::cout << "Robot State --------" << std::endl;
    std::cout << robot << std::endl;
    std::cout << "Attractor ----------" << std::endl;
    std::cout << attractor << std::endl;
    std::cout << "Commanded twist ----" << std::endl;
    std::cout << commandTwist << std::endl;
    std::cout << "Commanded torque ---" << std::endl;
    std::cout << commandTorque << std::endl;
    count = 0;
  }
  ++count;
}

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  auto attractor = state_representation::CartesianPose::Identity("attractor", "franka");
  state_representation::CartesianState robot("end-effector", "franka");
  state_representation::Jacobian jacobian("franka", 7, "end-effector", "franka");

  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  dynamical_systems::Linear<state_representation::CartesianState> DS(attractor, gains);

  controllers::impedance::CartesianTwistController ctrl(100, 100, 4, 4);

  network::Interface franka(network::InterfaceType::FRANKA_PAPA_16);
  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};
  command.controlType = frankalwi::proto::JOINT_TORQUE;
  network::Interface bridge(network::InterfaceType::BRIDGE);

  sensors::Joy joy(0.0001, 0.0005);
  joy.start();

  bool poseSet = false;
  // control loop
  while (franka.receive(state)) {
    frankalwi::utils::toCartesianState(state, robot);
    frankalwi::utils::toJacobian(state.jacobian, jacobian);
    if (!poseSet) {
      attractor.set_position(robot.get_position());
      attractor.set_orientation(robot.get_orientation());
      DS.set_attractor(attractor);
      poseSet = true;
    }

    auto joyPose = state_representation::CartesianPose::Identity(attractor.get_name(), attractor.get_reference_frame());
    joy.getJoyUpdate(joyPose);
    attractor += joyPose;
    DS.set_attractor(attractor);

    state_representation::CartesianTwist dsTwist = DS.evaluate(robot);
    dsTwist.clamp(0.25, 0.5);

    state_representation::JointTorques joint_command = ctrl.compute_command(dsTwist, robot, jacobian);

    frankalwi::utils::fromJointTorque(joint_command, command);
    throttledPrint(robot, attractor, dsTwist, joint_command, 500);

    franka.send(command);
    std::array<double, 7> attractorPose{};
    Eigen::MatrixXd::Map(&attractorPose[0], 7, 1) = attractor.get_pose().array();
    state.eePose = frankalwi::proto::EEPose(attractorPose);
    bridge.send(state);
  }
}
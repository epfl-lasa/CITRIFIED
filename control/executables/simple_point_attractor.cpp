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

  state_representation::CartesianPose attractor("attractor", "franka");
  state_representation::CartesianState robot("end-effector", "franka");
  state_representation::Jacobian jacobian("franka", 7, "end-effector", "franka");

  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  dynamical_systems::Linear<state_representation::CartesianState> DS(attractor, gains);

  controllers::impedance::CartesianTwistController ctrl(100, 100, 5, 5);

  bool positionSet = false;
  bool orientationSet = false;
  if (argc == 4) {
    attractor.set_position(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    std::cout << "Using target position from command line" << std::endl;
    positionSet = true;
  } else if (argc == 5) {
    std::cout << "Using target orientation from command line" << std::endl;
    Eigen::Quaterniond orientation(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]));
    attractor.set_orientation(orientation.normalized());
    orientationSet = true;
  } else if (argc == 8) {
    std::cout << "Using target pose from command line" << std::endl;
    attractor.set_position(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    Eigen::Quaterniond orientation(atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]));
    attractor.set_orientation(orientation.normalized());
    positionSet = true;
    orientationSet = true;
  }
  DS.set_attractor(attractor);

  network::Interface franka(network::InterfaceType::FRANKA_PAPA_16);
  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  sensors::Joy joy;
  joy.start();

  // control loop
  while (franka.receive(state)) {
    frankalwi::utils::toCartesianState(state, robot);
    frankalwi::utils::toJacobian(state.jacobian, jacobian);
    if (!positionSet || !orientationSet) {
      if (!positionSet) {
        std::cout << "Updating target position from current state" << std::endl;
        attractor.set_position(robot.get_position());
        positionSet = true;
      }
      if (!orientationSet) {
        std::cout << "Updating target orientation from current state" << std::endl;
        attractor.set_orientation(robot.get_orientation());
        orientationSet = true;
      }
      DS.set_attractor(attractor);
    }
    DS.set_attractor(DS.get_attractor() + joy.getJoyUpdate());

    state_representation::CartesianTwist dsTwist = DS.evaluate(robot);
    dsTwist.clamp(0.25, 0.5);

    state_representation::JointTorques joint_command = ctrl.compute_command(dsTwist, robot, jacobian);

    frankalwi::utils::fromJointTorque(joint_command, command);
    throttledPrint(robot, attractor, dsTwist, joint_command, 500);

    franka.send(command);
  }
}
#include <vector>
#include <iostream>

#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "controllers/CartesianPoseController.h"
#include "motion_generators/PointAttractorDS.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "network/interfaces.h"

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

  state_representation::CartesianPose attractor("attractor", "franka");
  state_representation::CartesianState robot("end-effector", "franka");
  state_representation::Jacobian jacobian("franka", 7);

  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  dynamical_systems::Linear<state_representation::CartesianState> DS(attractor, gains);

  controller::CartesianPoseController ctrl(50, 50, 5, 5);

  std::cout << std::fixed << std::setprecision(3);

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

  // Set up franka ZMQ
  network::Interface franka(network::InterfaceType::FRANKA_LWI);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

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

    state_representation::CartesianTwist dsTwist = DS.evaluate(robot);
    dsTwist.clamp(0.25, 0.5);
    auto torques = ctrl.getJointTorque(robot, dsTwist, jacobian);
    frankalwi::utils::fromJointTorque(torques, command);

    throttledPrint(robot, attractor, dsTwist, torques, 500);

    if (!franka.send(command)) {
      std::cerr << "Warning: Couldn't send command to Franka!" << std::endl;
    };
  }
}
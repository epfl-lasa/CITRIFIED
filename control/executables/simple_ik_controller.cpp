#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <dynamical_systems/Linear.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "controllers/KinematicController.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "franka_lwi/franka_lwi_logger.h"
#include "network/interfaces.h"

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // logger
  frankalwi::utils::Logger logger;

  // motion generator
  Eigen::Vector3d center = {0.35, 0, 0.5};
  Eigen::Quaterniond defaultOrientation = {0, 0.707, 0.707, 0};
  state_representation::CartesianPose attractor("attractor", center, defaultOrientation);

  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  dynamical_systems::Linear<state_representation::CartesianState> DS(attractor, gains);

  // controller
  controller::KinematicController ctrl;
  ctrl.gain = 10;

  // Set up franka ZMQ
  network::Interface franka(network::InterfaceType::FRANKA_PAPA_16);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  state_representation::CartesianState robot("robot");

  // control loop
  while (franka.receive(state)) {
    logger.writeLine(state);

    frankalwi::utils::toCartesianState(state, robot);
    state_representation::CartesianTwist twist = DS.evaluate(robot);
    twist.clamp(0.5, 1.0);
    // TODO this is just an intermediate solution
    std::vector<double> desiredVelocity = {
        twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
        twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
    };
    command = ctrl.getJointTorque(state, desiredVelocity);
    command.controlType = frankalwi::proto::JOINT_TORQUE;
    if (!franka.send(command)) {
      std::cerr << "Warning: Couldn't send command to Franka!" << std::endl;
    };
  }
}
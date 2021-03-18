#include <state_representation/space/cartesian/CartesianPose.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "controllers/KinematicController.h"
#include "motion_generators/PointAttractorDS.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "franka_lwi/franka_lwi_logger.h"
#include "network/interfaces.h"

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // logger
  frankalwi::utils::Logger logger;

  // motion generator
  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  Eigen::Vector3d center = {0.35, 0, 0.5};
  Eigen::Quaterniond defaultOrientation = {0, 0.707, 0.707, 0};

  state_representation::CartesianPose defaultPose("world", center, defaultOrientation);

  motion_generator::PointAttractor DS;
  DS.currentPose = defaultPose;
  DS.setTargetPose(DS.currentPose);
  DS.linearDS.set_gain(gains);

  // controller
  controller::KinematicController ctrl;
  ctrl.gain = 10;

  // Set up franka ZMQ
  network::Interface franka(network::InterfaceType::FRANKA_LWI);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  // control loop
  while (franka.receive(state)) {
    logger.writeLine(state);

    state_representation::CartesianPose pose(state_representation::CartesianPose::Identity("world"));
    frankalwi::utils::poseToState(state, pose);
    state_representation::CartesianTwist twist = DS.getTwist(pose);
    // TODO this is just an intermediate solution
    std::vector<double> desiredVelocity = {
        twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
        twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
    };
    command = ctrl.getJointTorque(state, desiredVelocity);
    if (!franka.send(command)) {
      std::cerr << "Warning: Couldn't send command to Franka!" << std::endl;
    };
  }
}
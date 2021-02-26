#include <state_representation/Space/Cartesian/CartesianPose.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "controllers/KinematicController.h"
#include "motion_generators/PointAttractorDS.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "franka_lwi/franka_lwi_logger.h"
#include "network/zmq_interface.h"

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // logger
  frankalwi::proto::Logger logger;

  // motion generator
  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  Eigen::Vector3d center = {0.35, 0, 0.5};
  Eigen::Quaterniond defaultOrientation = {0, 0.707, 0.707, 0};

  StateRepresentation::CartesianPose defaultPose("world", center, defaultOrientation);

  motion_generator::PointAttractor DS;
  DS.currentPose = defaultPose;
  DS.setTargetPose(DS.currentPose);
  DS.linearDS.set_gain(gains);

  // controller
  controller::KinematicController ctrl;
  ctrl.gain = 10;

  // communication
  zmq::context_t context;
  zmq::socket_t publisher, subscriber;
  network::zmq_interface::configureSockets(context, publisher, subscriber);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  // control loop
  bool stateReceived = false;
  while (subscriber.connected()) {
    if (network::zmq_interface::receive(subscriber, state)) {
      logger.writeLine(state);

      StateRepresentation::CartesianPose pose(StateRepresentation::CartesianPose::Identity("world"));
      frankalwi::proto::poseFromState(state, pose);
      StateRepresentation::CartesianTwist twist = DS.getTwist(pose);
      // TODO this is just an intermediate solution
      std::vector<double> desiredVelocity = {
          twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
          twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
      };
      command = ctrl.getJointTorque(state, desiredVelocity);
      network::zmq_interface::send(publisher, command);
    }
  }
}
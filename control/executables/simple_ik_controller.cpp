#include <state_representation/Space/Cartesian/CartesianPose.hpp>

#include "controllers/KinematicController.h"
#include "motion_generators/PointAttractorDS.h"
#include "network/netutils.h"
#include "franka_lwi/franka_lwi_logger.h"

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // logger
  frankalwi::proto::Logger logger;

  // motion generator
  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  Eigen::Vector3d center = {0.35, 0, 0.5};
  Eigen::Quaterniond defaultOrientation = {0, 0.707, 0.707, 0};

  StateRepresentation::CartesianPose defaultPose("world", center, defaultOrientation);

  motiongenerator::PointAttractor DS;
  DS.currentPose = defaultPose;
  DS.setTargetPose(DS.currentPose);
  DS.linearDS.set_gain(gains);

  // controller
  controller::KinematicController ctrl;
  ctrl.gain = 10;

  // communication
  zmq::context_t context;
  zmq::socket_t publisher, subscriber;
  network::configure(context, publisher, subscriber);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  // control loop
  bool stateReceived = false;
  while (subscriber.connected()) {
    if (frankalwi::proto::receive(subscriber, state)) {
      logger.writeLine(state);

      command = ctrl.getJointTorque(state, DS.getTwist(state));
      frankalwi::proto::send(publisher, command);
    }
  }
}
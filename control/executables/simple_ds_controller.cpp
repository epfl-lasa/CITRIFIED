#include <vector>
#include <iostream>
#include <cstdio>

#include <state_representation/Space/Cartesian/CartesianPose.hpp>
#include <state_representation/Space/Cartesian/CartesianState.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "controllers/CartesianPoseController.h"
#include "motion_generators/PointAttractorDS.h"
#include "network/netutils.h"

void throttledPrintCommand(const motiongenerator::PointAttractor& DS,
                           std::vector<double> velocity,
                           frankalwi::proto::CommandMessage<7> command,
                           int skip) {
  static int count = 0;
  if (count > skip) {

    std::cout << "COMMAND ------------" << std::endl;
    std::cout << "Target " << DS.targetPose << std::endl;

    printf("Desired twist:  % 3.2f, % 3.2f, % 3.2f, % 3.2f, % 3.2f, % 3.2f\n",
           velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]);
    printf("Desired torque: % 3.2f, % 3.2f, % 3.2f, % 3.2f, % 3.2f, % 3.2f, % 3.2f\n",
           command.jointTorque[0], command.jointTorque[1], command.jointTorque[2], command.jointTorque[3],
           command.jointTorque[4], command.jointTorque[5], command.jointTorque[6]);
    count = 0;
  }
  ++count;
}

void throttledPrintState(frankalwi::proto::StateMessage<7> state, int skip) {
  static int count = 0;
  if (count > skip) {

    std::cout << "STATE --------------" << std::endl;

    printf("State position xyz:     % 3.3f, % 3.3f, % 3.3f\n",
           state.eePose.position.x,
           state.eePose.position.y,
           state.eePose.position.z);
    printf("State orientation wxyz: % 3.3f, % 3.3f, % 3.3f, % 3.3f\n",
           state.eePose.orientation.w,
           state.eePose.orientation.x,
           state.eePose.orientation.y,
           state.eePose.orientation.z);

    const char map[3] = {'X', 'Y', 'Z'};
    for (std::size_t dof = 0; dof < 6; ++dof) {
      std::string space = dof < 3 ? "linear " : "angular";
      printf("Jacobian %s %c: ", space.c_str(), map[dof % 3]);
      for (std::size_t joint = 0; joint < 6; ++joint) {
        printf("% 5.2f, ", state.jacobian[dof][joint]);
      }
      printf("% 5.2f\n", state.jacobian[dof][6]);
    }
    count = 0;
  }
  ++count;
}

int main(int argc, char** argv) {
  motiongenerator::PointAttractor DS;
  DS.currentPose = StateRepresentation::CartesianPose::Identity("robot");
  DS.setTargetPose(DS.currentPose);

  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  DS.linearDS.set_gain(gains);

  controller::CartesianPoseController ctrl(20, 10, 10);

  std::cout << std::fixed << std::setprecision(3);

  bool positionSet = false;
  bool orientationSet = false;
  if (argc == 4) {
    DS.targetPose.set_position(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    std::cout << "Using target position from command line" << std::endl;
    positionSet = true;
    DS.setTargetPose(DS.targetPose);
  } else if (argc == 5) {
    std::cout << "Using target orientation from command line" << std::endl;
    Eigen::Quaterniond orientation(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]));
    DS.targetPose.set_orientation(orientation.normalized());
    orientationSet = true;
    DS.setTargetPose(DS.targetPose);
  } else if (argc == 8) {
    std::cout << "Using target pose from command line" << std::endl;
    DS.targetPose.set_position(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    Eigen::Quaterniond orientation(atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]));
    DS.targetPose.set_orientation(orientation.normalized());
    std::cout << DS.targetPose << std::endl;
    positionSet = true;
    orientationSet = true;
    DS.setTargetPose(DS.targetPose);
  }

  // Set up ZMQ
  zmq::context_t context;
  zmq::socket_t publisher, subscriber;
  network::configure(context, publisher, subscriber);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  while (subscriber.connected()) {
    // blocking receive until we get a state from the robot
    if (frankalwi::proto::receive(subscriber, state)) {
      if (!positionSet || !orientationSet) {
        if (!positionSet) {
          std::cout << "Updating target position from current state" << std::endl;
          DS.setTargetPosition(state);
          positionSet = true;
        }
        if (!orientationSet) {
          std::cout << "Updating target orientation from current state" << std::endl;
          DS.setTargetOrientation(state);
          orientationSet = true;
        }
        std::cout << DS.targetPose << std::endl;
      }

      std::vector<double> desiredVelocity = DS.getTwist(state);
      command = ctrl.getJointTorque(state, desiredVelocity);

      throttledPrintCommand(DS, desiredVelocity, command, 500);
      throttledPrintState(state, 500);

      frankalwi::proto::send(publisher, command);
    }
  }
}
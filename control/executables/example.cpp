#include <vector>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "motion_generators/PointAttractorDS.h"
#include "controllers/CartesianPoseController.h"

int main(int argc, char** argv) {
  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  motiongenerator::PointAttractor DS;
  controller::CartesianPoseController ctrl(1, 1, 1);

  DS.currentPose = StateRepresentation::CartesianPose::Identity("robot");
  DS.setTargetPose(DS.currentPose);

  std::vector<double> desiredVelocity = DS.getTwist(state);
  command = ctrl.getJointTorque(state, desiredVelocity);
}
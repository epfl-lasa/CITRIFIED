#include <state_representation/space/cartesian/CartesianState.hpp>
#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "network/interfaces.h"
#include "controllers/CartesianPoseController.h"
#include "motion_generators/PointAttractorDS.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "sensors/RigidBodyTracker.h"

int main(int argc, char** argv) {
  sensors::RigidBodyTracker tracker;
  tracker.start();
  int rigidBodyID = 1;

  state_representation::CartesianState rigidBodyState("rigid_body_1", "optitrack");
  state_representation::CartesianPose optiTrackInWorld("optitrack", Eigen::Vector3d(0.18, 0, 0.10), "world");
  state_representation::CartesianPose pose(state_representation::CartesianPose::Identity("robot", "world"));

  motion_generator::PointAttractor DS;
  DS.currentPose = pose;
  DS.setTargetPose(DS.currentPose);

  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  DS.linearDS.set_gain(gains);

  DS.maxLinearSpeed = 0.5;
  DS.maxAngularSpeed = 1.0;

  controller::CartesianPoseController ctrl(50, 50, 5);
  ctrl.angularController.setDamping(5);

  // Set up ZMQ
  network::Interface franka(network::InterfaceType::FRANKA_LWI);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  bool firstPose = true;
  while (franka.receive(state)) {
    frankalwi::utils::toCartesianPose(state, pose);
    if (firstPose) {
      DS.setTargetPose(pose);
      firstPose = false;
    } else if (tracker.getState(rigidBodyState, rigidBodyID)) {
      rigidBodyState *= state_representation::CartesianPose("offset", Eigen::Vector3d(0, 0, -0.1), "rigid_body_1");
      DS.setTargetPose(optiTrackInWorld * state_representation::CartesianPose(rigidBodyState));
    }

    state_representation::CartesianTwist twist = DS.getTwist(pose);

    std::vector<double> desiredVelocity = {
        twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
        twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
    };
    command = ctrl.getJointTorque(state, desiredVelocity);
    franka.send(command);
  }
}
#include <state_representation/Space/Cartesian/CartesianState.hpp>
#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "sensors/RigidBodyTracker.h"
#include "controllers/CartesianPoseController.h"
#include "motion_generators/PointAttractorDS.h"
#include "franka_lwi/franka_lwi_utils.h"

int main(int argc, char** argv) {
  sensors::RigidBodyTracker tracker(1);
  tracker.start();

  StateRepresentation::CartesianState rigidBodyState("rigid_body_1", "optitrack");
  StateRepresentation::CartesianPose optiTrackInWorld("optitrack", Eigen::Vector3d(0.18, 0, 0.10), "world");
  StateRepresentation::CartesianPose pose(StateRepresentation::CartesianPose::Identity("robot", "world"));

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
  zmq::context_t context;
  zmq::socket_t publisher, subscriber;
  frankalwi::utils::configureSockets(context, publisher, subscriber);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  bool firstPose = true;
  while (subscriber.connected()) {
    if (frankalwi::proto::receive(subscriber, state)) {
      frankalwi::utils::poseFromState(state, pose);
      if (firstPose) {
        DS.setTargetPose(pose);
        firstPose = false;
      } else if (tracker.getState(rigidBodyState)) {
        rigidBodyState *= StateRepresentation::CartesianPose("offset", Eigen::Vector3d(0, 0, -0.1), "rigid_body_1");
        DS.setTargetPose(optiTrackInWorld * StateRepresentation::CartesianPose(rigidBodyState));
      }

      StateRepresentation::CartesianTwist twist = DS.getTwist(pose);

      std::vector<double> desiredVelocity = {
          twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
          twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
      };
      command = ctrl.getJointTorque(state, desiredVelocity);
      frankalwi::proto::send(publisher, command);
    }
  }
}
#include <state_representation/Space/Cartesian/CartesianPose.hpp>

#include "controllers/CartesianPoseController.h"
#include "controllers/AdaptiveForceFollower.h"
#include "motion_generators/PointAttractorDS.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "franka_lwi/franka_lwi_logger.h"

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // logger
  frankalwi::proto::Logger logger;

  // motion generator
  std::vector<double> gains = {50.0, 0.0, 10.0, 10.0, 10.0, 10.0};
  Eigen::Vector3d center = {0.43, 0, 0.09};
  Eigen::Quaterniond defaultOrientation = {0, 0.707, 0.707, 0};
  StateRepresentation::CartesianPose defaultPose("world", center, defaultOrientation);

  motion_generator::PointAttractor DS;
  DS.currentPose = defaultPose;
  DS.setTargetPose(DS.currentPose);
  DS.linearDS.set_gain(gains);

  // controller
  double principleDamping = 100;
  double orthogonalDamping = 50;
  controller::CartesianPoseController ctrl(principleDamping, orthogonalDamping, 5);
  ctrl.angularController.setDamping(5);

  AdaptiveForceFollower AFF;
  AFF.defaultDamping = principleDamping;
  AFF.maxDamping = 200;
  AFF.minDamping = 10;
  AFF.adaptiveWeight = 0.75;

  // force
  Eigen::Vector3d desiredForce = {0, 5, -10};
  Eigen::Vector3d desiredVelocity = {0, 0.1, 0};

  // communication
  zmq::context_t context;
  zmq::socket_t publisher, subscriber;
  frankalwi::utils::configureSockets(context, publisher, subscriber);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  StateRepresentation::CartesianPose pose(StateRepresentation::CartesianPose::Identity("world"));
  Eigen::Vector3d adaptedVelocity;

  // control loop
  while (subscriber.connected()) {
    if (frankalwi::proto::receive(subscriber, state)) {
      frankalwi::utils::poseFromState(state, pose);

//      logger.writeLine(state);
      StateRepresentation::CartesianTwist twist = DS.getTwist(pose);
      twist.set_linear_velocity(twist.get_linear_velocity() + desiredVelocity);

      double ypos = pose.get_position().y();
      Eigen::Vector3d appliedForce;
      if (ypos >= -0.25 && ypos < -0.2) {
        appliedForce = desiredForce * (ypos + 0.25) / 0.05;
      } else if (ypos >= -0.2 && ypos < 0.2) {
        appliedForce = desiredForce;
      } else if (ypos >= 0.2 && ypos < 0.25) {
        appliedForce = desiredForce * (0.25 - ypos) / 0.05;
      } else {
        appliedForce.setZero();
      }

      // adapt damping and velocity
      AFF.update(appliedForce, twist.get_linear_velocity(), adaptedVelocity, principleDamping);
      ctrl.linearController.setDamping(principleDamping, orthogonalDamping);
      twist.set_linear_velocity(twist.get_linear_velocity() + adaptedVelocity);

      std::cout <<  adaptedVelocity.transpose() << " | " << principleDamping << std::endl;

      // calculate and apply wrench
      std::vector<double> vel = {
          twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
          twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
      };
      command = ctrl.getJointTorque(state, vel);
      frankalwi::proto::send(publisher, command);
    }
  }
}
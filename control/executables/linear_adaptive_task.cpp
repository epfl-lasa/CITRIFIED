#include <state_representation/Space/Cartesian/CartesianPose.hpp>

#include "controllers/CartesianPoseController.h"
#include "controllers/AdaptiveForceFollower.h"
#include "motion_generators/PointAttractorDS.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "franka_lwi/franka_lwi_logger.h"
#include "sensors/ForceTorqueSensor.h"

Eigen::Vector3d forceFunction(const StateRepresentation::CartesianPose& pose) {
  const Eigen::Vector3d desiredForce = {0, 0, -2.5};
//  double ypos = pose.get_position().y();
//  Eigen::Vector3d appliedForce;
//  if (ypos >= -0.12 && ypos < -0.11) {
//    appliedForce = desiredForce * (ypos + 0.12) / 0.01;
//  } else if (ypos >= -0.11 && ypos < 0.11) {
//    appliedForce = desiredForce;
//  } else if (ypos >= 0.11 && ypos < 0.12) {
//    appliedForce = desiredForce * (0.12 - ypos) / 0.01;
//  } else {
//    appliedForce = {0, 5, 0};
//  }

  return desiredForce;
}

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // logger
  frankalwi::proto::Logger logger("silicon_10_vertical_trial_adaptive_0.csv");

  // ft sensor
  sensors::ToolSpec tool;
  tool.mass = 0.08;
  tool.centerOfMass = Eigen::Vector3d(0, 0, 0.020);
  sensors::ForceTorqueSensor ft_sensor("ft_sensor", "128.178.145.89", tool, false, 100);
  StateRepresentation::CartesianWrench rawWrench("ft_sensor_raw", "ft_sensor");
  StateRepresentation::CartesianWrench wrench("ft_sensor", "ft_sensor");
  StateRepresentation::CartesianWrench bias("ft_sensor", "ft_sensor");


  // motion generator
  std::vector<double> gains = {25.0, 25.0, 0.0, 10.0, 10.0, 10.0};
  Eigen::Vector3d center = {0.445, -0.05, 0.300};
  Eigen::Quaterniond defaultOrientation = {0, 0.707, 0.707, 0}; // knife along world Y, fully vertical
//  Eigen::Quaterniond defaultOrientation = {0.183, 0.683, 0.683, -0.183}; // knife along world Y, 30 degrees tilt
  StateRepresentation::CartesianPose defaultPose("world", center, defaultOrientation);
  // additional constant velocity command
  Eigen::Vector3d desiredVelocity = {0, 0, -0.05};

  motion_generator::PointAttractor DS;
  DS.currentPose = defaultPose;
  DS.setTargetPose(DS.currentPose);
  DS.linearDS.set_gain(gains);

  // controller
  double principleDamping = 200;
  double orthogonalDamping = 50;
  controller::CartesianPoseController ctrl(principleDamping, orthogonalDamping, 5);
  ctrl.angularController.setDamping(5);

  AdaptiveForceFollower AFF;
  AFF.defaultDamping = principleDamping;
  AFF.maxDamping = 260;
  AFF.minDamping = 10;
  AFF.adaptiveWeight = 0.5;
  AFF.etol = 0.2;

  // force

  // communication
  zmq::context_t context;
  zmq::socket_t publisher, subscriber;
  frankalwi::utils::configureSockets(context, publisher, subscriber);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  StateRepresentation::CartesianPose pose(StateRepresentation::CartesianPose::Identity("world"));
  Eigen::Vector3d adaptedVelocity;


  auto start = std::chrono::system_clock::now();
  // control loop
  while (subscriber.connected()) {
    if (frankalwi::proto::receive(subscriber, state)) {
      frankalwi::utils::poseFromState(state, pose);

      if(pose.get_position().z() < 0.31) {
        std::cout << "Position exceeded threshold!" << std::endl;
        return 0;
      }

      // read the force sensor
      Eigen::Vector3d worldWrench;
      if (ft_sensor.computeBias(pose.get_orientation().toRotationMatrix(), 1000)) {
        ft_sensor.readBias(bias);
        ft_sensor.readContactWrench(wrench, pose.get_orientation().toRotationMatrix());
        worldWrench = pose.get_orientation().toRotationMatrix().transpose() * wrench.get_force();
      }
      std::cout << worldWrench.transpose() << std::endl;

      if(abs(worldWrench.z()) > 15) {
        std::cout << "Force exceeded threshold!" << std::endl;
        return 0;
      }

      // get the DS twist
      StateRepresentation::CartesianTwist twist = DS.getTwist(pose);
      twist.set_linear_velocity(twist.get_linear_velocity() + desiredVelocity);
      Eigen::VectorXd dsTwist(6);
      dsTwist << twist.get_linear_velocity(), twist.get_angular_velocity();

      // get the predicted / desired force vector
      Eigen::Vector3d appliedForce = forceFunction(pose);

      // adapt damping and velocity
      AFF.update(appliedForce, twist.get_linear_velocity(), adaptedVelocity, principleDamping);
      auto v = twist.get_linear_velocity();
      ctrl.linearController.setDamping(principleDamping, orthogonalDamping);
      twist.set_linear_velocity(twist.get_linear_velocity() + adaptedVelocity);

      // calculate and apply wrench
      std::vector<double> vel = {
          twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
          twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
      };
      command = ctrl.getJointTorque(state, vel);
      frankalwi::proto::send(publisher, command);

      // log the state, force and adaptive terms
      std::array<double, 6> arr{};
      std::copy_n(wrench.array().data(), size(arr), arr.begin());
      std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
      state.eeWrench = frankalwi::proto::EETwist(arr);
      logger.writeCustomLine(elapsed_seconds.count(), state,
                             std::vector<double>(dsTwist.data(), dsTwist.data() + dsTwist.size()),
                             std::vector<double>(appliedForce.data(), appliedForce.data() + appliedForce.size()),
                             std::vector<double>(adaptedVelocity.data(), adaptedVelocity.data() + adaptedVelocity.size()),
                             principleDamping);
    }
  }
}
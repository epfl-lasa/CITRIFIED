#include <state_representation/Space/Cartesian/CartesianState.hpp>
#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "network/interfaces.h"
#include "controllers/CartesianPoseController.h"
#include "motion_generators/PointAttractorDS.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "franka_lwi/franka_lwi_logger.h"
#include "sensors/RigidBodyTracker.h"

int main(int argc, char** argv) {
  sensors::RigidBodyTracker tracker;
  tracker.start();

  StateRepresentation::CartesianPose attractorInTask("attractor",
                                                     Eigen::Vector3d(0, 0, 0.2),
                                                     Eigen::Quaterniond(0, 1, 0, 0),
                                                     "task");
  StateRepresentation::CartesianState taskInOptitrack("task", "optitrack");
  StateRepresentation::CartesianState robotInOptitrack("robot", "optitrack");
  StateRepresentation::CartesianState probeInOptiTrack("probe", "optitrack");
  StateRepresentation::CartesianPose eeInRobot(StateRepresentation::CartesianPose::Identity("ee", "robot"));

  std::vector<double> gains = {50.0, 50.0, 0.0, 10.0, 10.0, 10.0};
  DynamicalSystems::Linear<StateRepresentation::CartesianState> DS(attractorInTask, gains);

  controller::CartesianPoseController ctrl(150, 100, 4);
  ctrl.angularController.setDamping(4);

  // Set up ZMQ
  network::Interface franka(network::InterfaceType::FRANKA_LWI);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  // wait for optitrack data
  while (!tracker.getState(robotInOptitrack, 1) || !tracker.getState(taskInOptitrack, 2)) {
  }
  std::cout << "Optitrack ready" << std::endl;

  std::ofstream outputFile;
  outputFile.open("/tmp/surface_probe_apple.csv", std::ofstream::out | std::ofstream::trunc);

  if (outputFile.is_open()) {
    outputFile << "time, base_x, base_y, base_z, base_qw, base_qx, base_qw, base_qz,";
    outputFile << "task_x, task_y, task_z, task_qw, task_qx, task_qw, task_qz";
    outputFile << "surface_x, surface_y, surface_z" << std::endl;
  }

  auto start = std::chrono::system_clock::now();
  int iterations = 0;
  while (franka.receive(state)) {
    frankalwi::utils::poseFromState(state, eeInRobot);
    tracker.getState(robotInOptitrack, 1);
    tracker.getState(taskInOptitrack, 2);
    tracker.getState(probeInOptiTrack, 3);

    auto taskInRobot = robotInOptitrack.inverse() * taskInOptitrack;
    DS.set_reference_frame(taskInRobot);
    StateRepresentation::CartesianTwist twist = DS.evaluate(eeInRobot);
//    twist.clamp(0.5, 0.0, 1.0, 0.0);

    std::vector<double> desiredVelocity = {
        twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
        twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
    };
    command = ctrl.getJointTorque(state, desiredVelocity);
//    franka.send(command);

    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;

    outputFile << elapsed_seconds.count() << ", ";
    for (std::size_t idx = 0; idx < 7; ++idx) {
      outputFile << robotInOptitrack.get_pose().data()[idx] << ", ";
    }
    for (std::size_t idx = 0; idx < 7; ++idx) {
      outputFile << taskInOptitrack.get_pose().data()[idx] << ", ";
    }
    outputFile << probeInOptiTrack.get_position().x() << ", ";
    outputFile << probeInOptiTrack.get_position().y() << ", ";
    outputFile << probeInOptiTrack.get_position().z() << std::endl;

//    ++iterations;
//    if (iterations > 1000) {
//      std::cout << double(1000) / elapsed_seconds.count() <<  " Hz" << std::endl;
//      start = std::chrono::system_clock::now();
//      iterations = 0;
//    }
  }
}
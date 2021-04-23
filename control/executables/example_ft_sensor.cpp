#include <iostream>
#include <cstdio>
#include <iomanip>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "sensors/ForceTorqueSensor.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "network/interfaces.h"

void throttledPrintWrench(const state_representation::CartesianWrench& wrench,
                          const state_representation::CartesianWrench& bias, int skip, double avg_freq) {
  static int count = 0;
  if (count > skip) {
    printf("Average frequency of wrench messages: % 3.3f\n", avg_freq);

    std::cout << "Wrench --------------" << std::endl;
    std::cout << wrench << std::endl;

//    std::cout << "Bias --------------" << std::endl;
//    std::cout << bias << std::endl;
    count = 0;
  }
  ++count;
}

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  sensors::ToolSpec tool;
  tool.mass = 0.08;
  tool.centerOfMass = Eigen::Vector3d(0, 0, 0.025);
  sensors::ForceTorqueSensor ft_sensor("ft_sensor", "128.178.145.248", 100, tool);
  state_representation::CartesianWrench rawWrench("ft_sensor_raw", "ft_sensor");
  state_representation::CartesianWrench wrench("ft_sensor", "ft_sensor");
  state_representation::CartesianWrench bias("ft_sensor", "ft_sensor");

  // Set up franka ZMQ
  network::Interface franka(network::InterfaceType::FRANKA_LWI);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  auto start = std::chrono::system_clock::now();
  int iterations = 0;
  while (franka.receive(state)) {
    state_representation::CartesianPose pose(state_representation::CartesianPose::Identity("world"));
    Eigen::Matrix3d worldToEERotation(pose.get_orientation().toRotationMatrix());
    frankalwi::utils::toCartesianPose(state, pose);
    // just for test purposes
    if (!ft_sensor.readRawData(rawWrench)) {
      std::cout << "getting raw data failed" << std::endl;
      break;
    }
    if (ft_sensor.computeBias(worldToEERotation, 200)) {
      ft_sensor.readBias(bias);
      ft_sensor.readContactWrench(wrench, worldToEERotation);
      // compute and send command here
    }
    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
    throttledPrintWrench(wrench, bias, 1000, iterations / elapsed_seconds.count());
    ++iterations;
  }
}
#include <iostream>
#include <cstdio>
#include <iomanip>

#include "sensors/ForceTorqueSensor.h"
#include "network/netutils.h"

void throttledPrintWrench(const StateRepresentation::CartesianWrench& wrench,
                          const StateRepresentation::CartesianWrench& bias,
                          int skip,
                          double avg_freq) {
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
  sensors::ForceTorqueSensor ft_sensor("ft_sensor", "192.168.1.1", tool, false, 100);
  StateRepresentation::CartesianWrench rawWrench("ft_sensor_raw", "ft_sensor");
  StateRepresentation::CartesianWrench wrench("ft_sensor", "ft_sensor");
  StateRepresentation::CartesianWrench bias("ft_sensor", "ft_sensor");

  zmq::context_t context;
  zmq::socket_t publisher, subscriber;
  network::configure(context, publisher, subscriber);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  auto start = std::chrono::system_clock::now();
  int iterations = 0;
  while (subscriber.connected()) {
    if (frankalwi::proto::receive(subscriber, state)) {
      StateRepresentation::CartesianPose pose(StateRepresentation::CartesianPose::Identity("world"));
      Eigen::Matrix3d worldToEERotation(pose.get_orientation().toRotationMatrix());
      network::poseFromState(state, pose);
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
}
#include <iostream>
#include <cstdio>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "network/interfaces.h"

int main(int argc, char** argv) {

  // Set up franka ZMQ
  network::Interface gripper(network::InterfaceType::FRANKA_QUEBEC_17_GRIPPER);

  frankalwi::proto::GripperStateMessage state;
  frankalwi::proto::GripperCommandMessage command;

  auto start = std::chrono::system_clock::now();
  bool received = false;
  int iterations;
  while (gripper.receive(state)) {
    if (!received) {
      received = true;
      start = std::chrono::system_clock::now();
      iterations = 0;
    }
    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
    std::cout << state.width << std::endl;

//    if (iterations == 0) {
//      command.stop = true;
//      gripper.send(command);
//    }
    if (iterations == 1) {
      command.stop = false;
      command.home = true;
      gripper.send(command);
    }
    if (iterations == 2) {
      command.home = false;
      command.width = 0.012;
      command.speed = 0.1;
      command.force = 20;
//      command.epsilon_outer = 0.001;
//      command.epsilon_inner = 0.001;
      gripper.send(command);
    }
//    if (!gripper.send(command)) {
//      std::cerr << "Warning: Couldn't send command to Franka!" << std::endl;
//    };
    ++iterations;
    std::cout << iterations << std::endl;
  }
}
#include <iostream>
#include <cstdio>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "network/interfaces.h"

int main(int argc, char** argv) {

  std::cout << std::fixed << std::setprecision(3);

  // Set up franka ZMQ
  network::Interface gpr(network::InterfaceType::GPR);

  double test = 1;
  double response;
  while (true) {
    gpr.send(test);
    if (gpr.poll(response)) {
      std::cout << response << std::endl;
    }
  }
}
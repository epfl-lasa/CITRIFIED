#include <iostream>
#include <array>

#include "network/interfaces.h"

int main(int argc, char** argv) {

  std::cout << std::fixed << std::setprecision(3);

  // Set up franka ZMQ
  network::Interface gpr(network::InterfaceType::GPR);

  std::array<double, 2> test = {0, 1};
  std::array<double, 2> response;
  while (true) {
    gpr.send(test);
    if (gpr.poll(response)) {
      std::cout << response[0] << ", " << response[1] << std::endl;
    }
  }
}
#include <iostream>
#include <unistd.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>

#include "sensors/Joy.h"

int main(int argc, char** argv) {
  sensors::Joy joy;
  joy.start();
  auto pose = state_representation::CartesianPose::Identity("joy", "world");
  while (true) {
    auto joyPose = state_representation::CartesianPose::Identity("joy", "world");
    joy.getJoyUpdate(joyPose);
    pose += joyPose;
    std::cout << pose << std::endl;
    usleep(1e6);
  }
}
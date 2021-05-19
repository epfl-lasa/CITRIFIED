#include <iostream>
#include <cstdio>

#include <network/zmq_joy_protocol.h>

#include "network/interfaces.h"
#include "sensors/Joy.h"

void throttledPrintState(int skip, double avg_freq) {
  static int count = 0;
  if (count > skip) {
    printf("Average frequency of state messages: % 3.3f\n", avg_freq);
    count = 0;
  }
  ++count;
}

int main(int argc, char** argv) {

//  // Set up franka ZMQ
//  network::Interface joy(network::InterfaceType::JOY);
//  pure_joy::proto::JoyMessage joyMessage{};
//
//  auto start = std::chrono::system_clock::now();
//  int iterations;
//  bool received = false;
//  while (joy.receive(joyMessage)) {
//    if (!received) {
//      received = true;
//      start = std::chrono::system_clock::now();
//      iterations = 0;
//    }
//    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
////    std::cout << joyMessage.axes[0] << std::endl;
//    throttledPrintState(500, iterations / elapsed_seconds.count());
//    ++iterations;
//  }
  sensors::Joy joy;
  joy.start();
  while (true) {
    std::cout << joy.getJoyUpdate() << std::endl;
  }
}
#include <iostream>
#include <array>

#include "learning/GPR.h"

int main(int, char**) {
  // either python test_interface.py or python gpr_server.py but rbf_orange_0503.pickle needed in script folder
  const int inputDims = 2;
  learning::GPR<inputDims> gpr;
  std::cout << "Waiting for GPR server..." << std::endl;
  gpr.testConnection();
  std::cout << "GPR server ready" << std::endl;

  std::cout << "Loading GPR file..." << std::endl;
  if(!gpr.start(2)) {
    std::cout << "Couldn't load GPR for this class." << std::endl;
    return 0;
  }
  // pre-load the GPR model by sending a dummy packet
  gpr.updateState(std::array<double, inputDims>{0, 0});

  int counter = 0;
  auto start = std::chrono::system_clock::now();
  for (std::size_t i = 0; i < 100; ++i) {
    std::array<double, 2> request = {static_cast<double>(i), static_cast<double>(i)};
    gpr.updateState(request);
    usleep(4000);
    if (auto prediction = gpr.getLastPrediction()) {
      ++counter;
      std::cout << "mean: " << prediction->mean << ", std: " << prediction->sigma << std::endl;
    }
  }
  gpr.stop();
  std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
  std::cout << elapsed_seconds.count() / 100 - 0.002 << std::endl;
  std::cout << counter << std::endl;
}
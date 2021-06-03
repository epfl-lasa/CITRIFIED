#include <iostream>
#include <array>

#include "learning/GPR.h"

int main(int, char**) {
  learning::GPR<2> gpr;
  std::string fruit = "orange";
  int counter = 0;
  if (!gpr.start(2)) {
    std::cout << "Couldn't start GPR thread" << std::endl;
    exit(1);
  };
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
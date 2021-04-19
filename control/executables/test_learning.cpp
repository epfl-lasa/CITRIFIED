#include <iostream>
#include <array>

#include "learning/GPR.h"

int main(int, char**) {
  learning::GPR gpr(2);
  std::string fruit = "orange";
  gpr.initialize(fruit);
  gpr.initialize(fruit);
  for (std::size_t i = 0; i < 15; ++i) {
    std::array<double, 2> request = {static_cast<double>(i), static_cast<double>(i)};
    gpr.predict(request);
    if (auto prediction = gpr.latestPrediction()) {
      std::cout << "mean: " << prediction->mean << ", std: " << prediction->sigma << std::endl;
    }
  }
}
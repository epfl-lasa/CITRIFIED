#include "learning/ESN.h"
#include <iostream>
#include <chrono>

int main() {
  learning::ESN esn(ESN_FILEPATH);
  esn.printAll();

  Eigen::MatrixXd signal = Eigen::Matrix<double, 97, 6>::Ones() * 2;
  std::cout << esn.test_esn(signal, 3);

  auto start = std::chrono::system_clock::now();
  for (int it = 0; it < 100; ++it) {
    esn.test_esn(signal, 6);
  }
  std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
  std::cout << "time for 100 evaluations: " << elapsed_seconds.count() << std::endl;
}
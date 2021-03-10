#include "learning/ESN.h"
#include <iostream>

int main() {
  learning::ESN esn("/home/dominic/git/esn-filter-tests/cpp/test_esn2.txt");
//  esn.printAll();

  Eigen::MatrixXd signal = Eigen::Matrix<double, 97, 9>::Ones() * 2;
  std::cout << esn.test_esn(signal, 3);

}
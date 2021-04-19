#pragma once

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>
#include <iostream>

#include "network/interfaces.h"

namespace learning {

struct gprPrediction {
  double mean = 99;
  double sigma = 99;
};

class GPR {
public:
  explicit GPR(const int& inputDim);

  ~GPR() = default;

  bool initialize(const std::string& className);

  template<std::size_t inputDim>
  bool predict(const std::array<double, inputDim>& state);

  std::optional<gprPrediction> latestPrediction();

private:
  const int inputDim_;
  network::Interface interface_;
  bool ready_ = false;
  bool received_ = false;
  gprPrediction prediction_;
};

template<std::size_t inputDim>
bool GPR::predict(const std::array<double, inputDim>& state) {
  if (!ready_) {
    std::cout << "initialize first" << std::endl;
    return false;
  }
  if (state.size() != inputDim_) {
    std::cout << "state has wrong size" << std::endl;
    return false;
  }
  interface_.send(state);
  gprPrediction prediction;
  received_ = interface_.receive(prediction);
  prediction_ = prediction;
  return received_;
}

}
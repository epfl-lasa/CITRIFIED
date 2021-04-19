#pragma once

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>

#include "network/interfaces.h"

namespace learning {

//template<std::size_t inputDim>
//struct gprState {
//  double& operator[](std::size_t i) {
//    return data[i % inputDim];
//  }
//  std::array<double, inputDim> data;
//};

struct gprPrediction {
  double mean = 99;
  double sigma = 99;
};

class GPR {
public:
  explicit GPR(const int& inputDim);

  ~GPR() = default;

  bool initialize(const std::string& className);

//  template<std::size_t inputDim>
  bool predict(const std::array<double, 2>& state);

  std::optional<gprPrediction> latestPrediction();

private:
  network::Interface interface_;
  bool ready_ = false;
  bool received_ = false;

  gprPrediction prediction_;

  const int inputDim_;
};
}
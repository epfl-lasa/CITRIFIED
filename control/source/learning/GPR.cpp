#include "learning/GPR.h"

#include <iostream>

namespace learning {

GPR::GPR(const int& inputDim) : inputDim_(inputDim), interface_(network::InterfaceType::GPR) {}

bool GPR::initialize(const std::string& className) {
  if (ready_) {
    std::cout << "already ready" << std::endl;
    return ready_;
  }
  interface_.send_string(className);
  interface_.receive(ready_);
  if (ready_) { std::cout << "ready" << std::endl; }
  else { std::cout << "not ready" << std::endl; }
  return ready_;
}

//template<std::size_t inputDim>
//void GPR::predict(const gprState<inputDim>& state) {
bool GPR::predict(const std::array<double, 2>& state) {
  if (!ready_) {
    std::cout << "initialize first" << std::endl;
    return false;
  }
  interface_.send(state);
  gprPrediction prediction;
  received_ = interface_.receive(prediction);
  prediction_ = prediction;
  return received_;
//  return prediction;
}

std::optional<gprPrediction> GPR::latestPrediction() {
  if (ready_ && received_) {
    return prediction_;
  }
}

}
#include "learning/GPR.h"

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

std::optional<gprPrediction> GPR::latestPrediction() {
  if (ready_ && received_) {
    return prediction_;
  }
  return {};
}

}
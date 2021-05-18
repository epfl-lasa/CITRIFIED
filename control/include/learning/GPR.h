#pragma once

#include <string>
#include <array>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <iostream>

#include "network/interfaces.h"

namespace learning {

struct gprInitMessage {
  int header = 0;
  int classIndex;
};

template<int inputDim>
struct gprStateMessage {
  int header = 1;
  std::array<double, inputDim> state;
};

struct gprPrediction {
  double mean = 99;
  double sigma = 99;

  std::vector<double> data() { return std::vector<double>{mean, sigma}; };
};

template<int inputDim>
class GPR {
public:
  GPR() = default;

  ~GPR() = default;

  bool start(const int& classIndex);
  void stop();
  bool testConnection();

  bool updateState(const std::array<double, inputDim>& state);

  std::optional<gprPrediction> predict();

  std::optional<gprPrediction> getLastPrediction();

private:
  int inputDim_ = 0;
  network::Interface interface_ = network::Interface(network::InterfaceType::GPR);
  gprStateMessage<inputDim> state_;

  std::thread gprThread_;
  std::mutex gprMutex_;
  bool keepAlive_ = false;

  bool stateReady_ = false;
  bool predictionReady_ = false;
  gprPrediction prediction_;

  void runPredictor();
};

template<int inputDim>
bool GPR<inputDim>::start(const int& classIndex) {
  inputDim_ = inputDim;
  gprInitMessage message{0, classIndex};
  interface_.send(message);
  bool success = false;
  interface_.receive(success);
  if (success && !keepAlive_) {
    keepAlive_ = true;
    gprThread_ = std::thread([this] { runPredictor(); });
  }
  return success;
}

template<int inputDim>
void GPR<inputDim>::stop() {
  keepAlive_ = false;
  gprThread_.join();
}

template<int inputDim>
void GPR<inputDim>::runPredictor() {
  while (keepAlive_) {
    predict();
    usleep(1000);
  }
}

template<int inputDim>
bool GPR<inputDim>::testConnection() {
  gprInitMessage message{99, 0};
  interface_.send(message);
  bool success;
  interface_.template receive(success);
  return success;
}

template<int inputDim>
std::optional<gprPrediction> GPR<inputDim>::getLastPrediction() {
  if (!predictionReady_) {
    return {};
  }
  predictionReady_ = false;
  return prediction_;
}

template<int inputDim>
bool GPR<inputDim>::updateState(const std::array<double, inputDim>& state) {
  if (state.size() != inputDim_) {
    std::cout << "[GPR::updateState] The input has a wrong size: " << state.size() << "!=" << inputDim_ << std::endl;
    return false;
  }
  gprMutex_.lock();
  state_.state = state;
  stateReady_ = true;
  gprMutex_.unlock();
  return true;
}

template<int inputDim>
std::optional<gprPrediction> GPR<inputDim>::predict() {
  if (!stateReady_) {
    return {};
  }
  gprMutex_.lock();
  gprStateMessage<inputDim> stateCopy = state_;
  gprMutex_.unlock();
  interface_.send(stateCopy);
  gprPrediction prediction;
  predictionReady_ = interface_.receive(prediction);
  if (predictionReady_) {
    prediction_ = prediction;
    return prediction_;
  } else {
    return {};
  }
}

}
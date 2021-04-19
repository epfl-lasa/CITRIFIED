#pragma once

#include <optional>
#include <thread>
#include <mutex>
#include <chrono>

#include "learning/ESN.h"

namespace learning {

class ESNWrapper {
public:
  explicit ESNWrapper(const std::string& esnConfigFile, int bufferSize, double minMillisecondsBetweenTimeWindows = 0);
  void setDerivativeCalculationIndices(const std::vector<int>& indices);

  void start();
  void stop();

  std::optional<esnPrediction> getLastPrediction();
  std::optional<esnPrediction> getLastPrediction(Eigen::MatrixXd& timeBuffer, Eigen::MatrixXd& dataBuffer);

  void addSample(double time, const Eigen::VectorXd& sample);

  std::optional<esnPrediction> classify();

  std::string getFinalClass(const std::vector<learning::esnPrediction>& predictionCollection);

  int predictionSplits = 3;

private:
  ESN esn_;
  Eigen::MatrixXd timeBuffer_, dataBuffer_, timeBufferCopy_, dataBufferCopy_;
  std::vector<int> derivativeIndices_;

  std::thread esnThread_;
  std::mutex esnMutex_;
  bool keepAlive_ = false;

  bool predictionReady_ = false;
  esnPrediction prediction_;

  int inputDimensions_;
  int bufferSize_;
  int bufferedSamples_ = 0;

  double minSecondsBetweenTimeWindows_;
  std::chrono::time_point<std::chrono::system_clock> lastPredictionTriggered_;

  void runClassifier();

  void calculateDerivatives();
  [[nodiscard]] bool dataBufferReady() const;
};

}
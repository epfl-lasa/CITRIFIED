#include "learning/ESNWrapper.h"

#include <unistd.h>

namespace learning {

ESNWrapper::ESNWrapper(const std::string& esnConfigFile, int bufferSize, double minMillisecondsBetweenTimeWindows) :
    esn_(esnConfigFile),
    bufferSize_(bufferSize),
    minSecondsBetweenTimeWindows_(minMillisecondsBetweenTimeWindows / 1000),
    lastPredictionTriggered_(std::chrono::system_clock::now()) {
  inputDimensions_ = esn_.inputDimensions();
  dataBuffer_ = Eigen::MatrixXd(bufferSize_, inputDimensions_);
  dataBuffer_.setZero();
  dataBufferCopy_ = dataBuffer_;

  timeBuffer_ = Eigen::MatrixXd(bufferSize_, 1);
  timeBuffer_.setZero();
  timeBufferCopy_ = timeBuffer_;
}

void ESNWrapper::setDerivativeCalculationIndices(const std::vector<int>& indices) {
  derivativeIndices_ = indices;
}

void ESNWrapper::start() {
  keepAlive_ = true;
  esnThread_ = std::thread([this] { runClassifier(); });
}

void ESNWrapper::stop() {
  keepAlive_ = false;
//  esnThread_.join();
}

std::optional<esnPrediction> ESNWrapper::getLastPrediction() {
  if (!predictionReady_) {
    return {};
  }
  predictionReady_ = false;
  return prediction_;
}

std::optional<esnPrediction> ESNWrapper::getLastPrediction(Eigen::MatrixXd& timeBuffer, Eigen::MatrixXd& dataBuffer) {
  auto prediction = getLastPrediction();
  if (prediction) {
    dataBuffer = dataBufferCopy_;
    timeBuffer = timeBufferCopy_;
  }
  return prediction;
}

void ESNWrapper::addSample(double time, const Eigen::VectorXd& sample) {
  // shift the time and sample observations up one row, and append the most recent sample to the end
  esnMutex_.lock();
  timeBuffer_.block(0, 0, bufferSize_ - 1, 1) = timeBuffer_.block(1, 0, bufferSize_ - 1, 1);
  dataBuffer_.block(0, 0, bufferSize_ - 1, sample.size()) = dataBuffer_.block(1, 0, bufferSize_ - 1, sample.size());

  timeBuffer_(bufferSize_ - 1) = time;
  dataBuffer_.block(bufferSize_ - 1, 0, 1, sample.size()) = sample.transpose();
  esnMutex_.unlock();

  if (bufferedSamples_ < bufferSize_) {
    ++bufferedSamples_;
  }
}

std::optional<esnPrediction> ESNWrapper::classify() {
  std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - lastPredictionTriggered_;
  if (!dataBufferReady() || elapsed_seconds.count() < minSecondsBetweenTimeWindows_) {
    return {};
  }

  esnMutex_.lock();
  dataBufferCopy_ = dataBuffer_;
  timeBufferCopy_ = timeBuffer_;
  lastPredictionTriggered_ = std::chrono::system_clock::now();
  esnMutex_.unlock();

  calculateDerivatives();

  prediction_ = esn_.predict(dataBufferCopy_, predictionSplits);
  predictionReady_ = true;
  return prediction_;
}

void ESNWrapper::runClassifier() {
  while (keepAlive_) {
    classify();
    usleep(1000);
  }
}

void ESNWrapper::calculateDerivatives() {
  int n = derivativeIndices_.size();
  if (n < 1) {
    return;
  }

  auto diff = timeBuffer_.block(1, 0, bufferSize_ - 1, 1) - timeBuffer_.block(0, 0, bufferSize_ - 1, 1);
  double dt = diff.mean();

  Eigen::MatrixXd signals(bufferSize_, n);
  for (int index = 0; index < derivativeIndices_.size(); ++index) {
    signals.block(0, index, bufferSize_, 1) = dataBufferCopy_.col(derivativeIndices_[index]);
  }

  dataBufferCopy_.block(1, inputDimensions_ - n, bufferSize_ - 2, n) = (signals.block(2, 0, bufferSize_ - 2, n) - signals.block(0, 0, bufferSize_ - 2, n)) / (2 * dt);
  dataBufferCopy_.block(0, inputDimensions_ - n, 1, n) = dataBufferCopy_.block(1, inputDimensions_ - n, 1, n);
  dataBufferCopy_.block(bufferSize_ - 1, inputDimensions_ - n, 1, n) = dataBufferCopy_.block(bufferSize_ - 2, inputDimensions_ - n, 1, n);
}

bool ESNWrapper::dataBufferReady() const {
  return bufferedSamples_ >= bufferSize_;
}

esnPrediction ESNWrapper::getFinalClass(const std::vector<learning::esnPrediction>& predictionCollection) const {
  Eigen::VectorXd sumOfProbabilities = Eigen::VectorXd::Zero(predictionCollection.at(0).predictions.size());
  for (const auto& prediction : predictionCollection) {
    sumOfProbabilities += prediction.predictions;
  }
  Eigen::MatrixXd::Index maxIndex;
  sumOfProbabilities.maxCoeff(&maxIndex);
  esnPrediction prediction = {esn_.classNames().at(maxIndex), static_cast<int>(maxIndex), sumOfProbabilities};
  return prediction;
}

}

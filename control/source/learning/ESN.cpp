#include "learning/ESN.h"

#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace learning {

ESN::ESN(const std::string& file) {
  YAML::Node esnParams = YAML::LoadFile(file);

  readInt(esnParams, "nForgetPoints", nbForgetPoints_);
  readInt(esnParams, "nInternalUnits", nbInternalUnits_);
  readInt(esnParams, "nInputUnits", nbInputs_);
  readInt(esnParams, "nOutputUnits", nbOutputs_);
  try {
    classNames_ = esnParams["classNames"].as<std::vector<std::string>>();
  } catch (YAML::Exception& ex) {
    std::cerr << ex.what() << std::endl;
    throw std::invalid_argument("[ESN] Error parsing parameter. Could not find 'classNames'!");
  }

  readMatrix(esnParams, "inputScaling", inputScaling_, nbInputs_);
  readMatrix(esnParams, "inputShift", inputShift_, nbInputs_);

  Eigen::MatrixXd inputWeights, internalWeights, feedbackWeights;
  readMatrix(esnParams, "inputWeights", inputWeights, nbInternalUnits_, nbInputs_);
  readMatrix(esnParams, "internalWeights", internalWeights, nbInternalUnits_, nbInternalUnits_);
  readMatrix(esnParams, "feedbackWeights", feedbackWeights, nbInternalUnits_, nbOutputs_);
  readMatrix(esnParams, "feedbackScaling", feedbackScaling_, nbOutputs_);
  systemEquationWeightMatrix_ = Eigen::MatrixXd::Zero(nbInternalUnits_, nbInternalUnits_ + nbInputs_ + nbOutputs_);
  systemEquationWeightMatrix_.topLeftCorner(nbInternalUnits_, nbInternalUnits_) = internalWeights;
  systemEquationWeightMatrix_.block(0, nbInternalUnits_, nbInternalUnits_, nbInputs_) = inputWeights;
  systemEquationWeightMatrix_.topRightCorner(nbInternalUnits_, nbOutputs_) =
      feedbackWeights * feedbackScaling_.asDiagonal();

  readMatrix(esnParams, "outputWeights", outputWeights_, nbOutputs_, nbInternalUnits_ + nbInputs_);
  readMatrix(esnParams, "teacherShift", teacherShift_, nbOutputs_);
  readMatrix(esnParams, "teacherScaling", teacherScaling_, nbOutputs_);
  invTeacherScalingMatrix_ = teacherScaling_.asDiagonal().inverse();;
}

void ESN::readInt(const YAML::Node& params, const std::string& paramName, int& var) {
  try {
    var = params[paramName].as<int>();
  } catch (YAML::Exception& ex) {
    std::cerr << ex.what() << std::endl;
    throw std::invalid_argument("[ESN] Error parsing parameter. Could not find '" + paramName + "'!");
  }
}
template <typename T>
void ESN::readMatrix(const YAML::Node& params, const std::string& paramName, T& matrix, const int& rows,
                     const int& cols) {
  try {
    auto tmp = params[paramName].as<std::vector<double>>();
    if (tmp.size() != rows * cols) {
      std::cerr << "[ESN] Warning! Parameter '" + paramName + "' does not have the expected length!" << std::endl;
    }
    matrix = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(tmp.data(), rows, cols);
  } catch (YAML::Exception& ex) {
    std::cerr << ex.what() << std::endl;
    throw std::invalid_argument("[ESN] Error parsing parameter. Could not find '" + paramName + "'!");
  }
}

void ESN::printAll() {
  std::cout << "number of Internal Units: " << nbInternalUnits_ << std::endl;
  std::cout << "number of Input Units: " << nbInputs_ << std::endl;
  std::cout << "number of Output Units: " << nbOutputs_ << std::endl;

  std::cout << "class Names: ";
  for (const auto& c : classNames_) {
    std::cout << c << " ";
  }
  std::cout << std::endl;

  std::cout << "input Scaling:" << std::endl;
  std::cout << inputScaling_.transpose() << std::endl;

  std::cout << "input Shift:" << std::endl;
  std::cout << inputShift_.transpose() << std::endl;

  std::cout << "teacher Scaling:" << std::endl;
  std::cout << teacherScaling_.transpose() << std::endl;

  std::cout << "teacher Shift:" << std::endl;
  std::cout << teacherShift_.transpose() << std::endl;

  std::cout << "feedback Scaling:" << std::endl;
  std::cout << feedbackScaling_.transpose() << std::endl;
}

esnPrediction ESN::predict(const Eigen::MatrixXd& data, const int& nbPredictionSplits) {
  if (data.cols() != nbInputs_) {
    throw std::invalid_argument(
        "[ESN::predict] The number of columns in function input 'data' does not match the expected number of inputs!");
  }
  Eigen::MatrixXd stateCollectionMatrix = collectStates(data);

  Eigen::MatrixXd outputSequence = stateCollectionMatrix * outputWeights_.transpose();
  outputSequence = (outputSequence.rowwise() - teacherShift_.transpose()) * invTeacherScalingMatrix_;

  return classify(outputSequence, nbPredictionSplits);
}

Eigen::MatrixXd ESN::collectStates(const Eigen::MatrixXd& signal) {
  Eigen::VectorXd totalState = Eigen::VectorXd::Zero(nbInternalUnits_ + nbInputs_ + nbOutputs_);
  Eigen::MatrixXd
      stateCollection = Eigen::MatrixXd::Zero(signal.rows() - nbForgetPoints_, nbInternalUnits_ + nbInputs_);

  for (int i = nbForgetPoints_; i < signal.rows(); i++) {
    totalState.segment(nbInternalUnits_, nbInputs_) =
        inputScaling_.cwiseProduct(signal.row(i).transpose()) + inputShift_;
    totalState.head(nbInternalUnits_) = (systemEquationWeightMatrix_ * totalState).unaryExpr(&tanh);
    totalState.tail(nbOutputs_) = outputWeights_ * totalState.head(nbInternalUnits_ + nbInputs_);
    stateCollection.row(i - nbForgetPoints_) = totalState.head(nbInternalUnits_ + nbInputs_);
  }
  return stateCollection;
}

esnPrediction ESN::classify(const Eigen::MatrixXd& outputSeq, const int& nbSplits) const {
  int split = outputSeq.rows() / nbSplits;
  Eigen::MatrixXd averagePredictedOutput = Eigen::MatrixXd::Zero(nbSplits, nbOutputs_);
  for (int i = 0; i < nbSplits - 1; ++i) {
    averagePredictedOutput.row(i) = outputSeq.middleRows(i * split, split).colwise().mean();
  }
  averagePredictedOutput.row(nbSplits - 1) =
      outputSeq.bottomRows(outputSeq.rows() - (nbSplits - 1) * split).colwise().mean();
  Eigen::VectorXd sumOfAveragePredictedOutput =
      averagePredictedOutput.colwise().sum() / averagePredictedOutput.colwise().sum().sum();

  esnPrediction result;
  Eigen::MatrixXd::Index maxIndex;
  sumOfAveragePredictedOutput.maxCoeff(&maxIndex);
  result.classIndex = maxIndex;
  result.className = classNames_.at(maxIndex);
  result.predictions = sumOfAveragePredictedOutput;

  return result;
}

esnPrediction ESN::classify_softmax(const Eigen::MatrixXd& outputSeq, const int& nbSplits) const {
  int split = outputSeq.rows() / nbSplits;
  Eigen::MatrixXd averagePredictedOutput = Eigen::MatrixXd::Zero(nbSplits, nbOutputs_);
  for (int i = 0; i < nbSplits - 1; ++i) {
    averagePredictedOutput.row(i) = outputSeq.middleRows(i * split, split).colwise().mean();
  }
  averagePredictedOutput.row(nbSplits - 1) =
      outputSeq.bottomRows(outputSeq.rows() - (nbSplits - 1) * split).colwise().mean();
  Eigen::VectorXd sumOfAveragePredictedOutput = averagePredictedOutput.colwise().sum();
  double softmaxSum = sumOfAveragePredictedOutput.unaryExpr(&exp).sum();
  for (int i = 0; i < nbOutputs_; ++i) {
    sumOfAveragePredictedOutput(i) = exp(sumOfAveragePredictedOutput(i)) / softmaxSum;
  }

  esnPrediction result;
  Eigen::MatrixXd::Index maxIndex;
  sumOfAveragePredictedOutput.maxCoeff(&maxIndex);
  result.classIndex = maxIndex;
  result.className = classNames_.at(maxIndex);
  result.predictions = sumOfAveragePredictedOutput;

  return result;
}

int ESN::inputDimensions() const {
  return nbInputs_;
}

std::vector<std::string> ESN::classNames() const {
  return classNames_;
}

}
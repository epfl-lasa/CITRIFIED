#include "learning/ESN.h"

#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace learning {

ESN::ESN(const std::string& file) {
  YAML::Node esnParams = YAML::LoadFile(file);

  nbForgetPoints_ = esnParams["nForgetPoints"].as<int>();
  nbInternalUnits_ = esnParams["nInternalUnits"].as<int>();
  nbInputs_ = esnParams["nInputUnits"].as<int>();
  nbOutputs_ = esnParams["nOutputUnits"].as<int>();
  nbTotalUnits_ = esnParams["nTotalUnits"].as<int>();

  readMatrix(esnParams, "inputWeights", inputWeights_, nbInternalUnits_, nbInputs_);
  readMatrix(esnParams, "outputWeights", outputWeights_, nbOutputs_, nbInternalUnits_ + nbInputs_);
  readMatrix(esnParams, "feedbackWeights", feedbackWeights_, nbInternalUnits_, nbOutputs_);
  readMatrix(esnParams, "internalWeights", internalWeights_, nbInternalUnits_, nbInternalUnits_);

  readVector(esnParams, "inputScaling", inputScaling_, nbInputs_);
  readVector(esnParams, "inputShift", inputShift_, nbInputs_);
  readVector(esnParams, "teacherScaling", teacherScaling_, nbOutputs_);
  readVector(esnParams, "teacherShift", teacherShift_, nbOutputs_);
  readVector(esnParams, "feedbackScaling", feedbackScaling_, nbOutputs_);
}

void ESN::readMatrix(const YAML::Node& params, const std::string& paramName, Eigen::MatrixXd& matrix, const int& rows,
                     const int& cols) {
  try {
    auto tmp = params[paramName].as < std::vector < double >> ();
    if (tmp.size() != rows * cols) {
      std::cerr << "[ESN] Warning! Parameter '" + paramName + "' does not have the expected length!" << std::endl;
    }
    matrix = Eigen::Map < Eigen::Matrix < double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor
        >> (tmp.data(), rows, cols);
  } catch (YAML::Exception& ex) {
    std::cerr << ex.what() << std::endl;
    throw std::invalid_argument("[ESN] Error parsing parameter. Could not find '" + paramName + "'!");
  }
}

void ESN::readVector(const YAML::Node& params, const std::string& paramName, Eigen::VectorXd& vector, const int& rows) {
  try {
    auto tmp = params[paramName].as < std::vector < double >> ();
    if (tmp.size() != rows) {
      std::cerr << "[ESN] Warning! Parameter '" + paramName + "' does not have the expected length!" << std::endl;
    }
    vector = Eigen::Map<Eigen::VectorXd>(tmp.data(), rows);
  } catch (YAML::Exception& ex) {
    std::cerr << ex.what() << std::endl;
    throw std::invalid_argument("[ESN] Error parsing parameter. Could not find '" + paramName + "'!");
  }
}

void ESN::printAll() {
  std::cout << "number of Internal Units: " << nbInternalUnits_ << std::endl;
  std::cout << "number of Input Units: " << nbInputs_ << std::endl;
  std::cout << "number of Output Units: " << nbOutputs_ << std::endl;

  std::cout << "number of Total Units: " << nbTotalUnits_ << std::endl;

  std::cout << "input Weights:" << std::endl;
//  std::cout << inputWeights_ << std::endl;

  std::cout << "output Weights:" << std::endl;
//  std::cout << outputWeights_ << std::endl;

  std::cout << "feedback Weights:" << std::endl;
//  std::cout << feedbackWeights_ << std::endl;

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

  std::cout << "internal Weights:" << std::endl;
//  std::cout << internalWeights_ << std::endl;
}

int ESN::test_esn(const Eigen::MatrixXd& data, int nbDataPoints) {
  Eigen::MatrixXd stateCollection = compute_statematrix(data, nbDataPoints);

  Eigen::MatrixXd outputSequence = stateCollection * outputWeights_.transpose();
  outputSequence -= teacherShift_.transpose().replicate(nbDataPoints - nbForgetPoints_, 1);
  outputSequence *= teacherScaling_.asDiagonal().inverse();

  Eigen::VectorXd classification = s_classify2(outputSequence, 3);

  Eigen::VectorXd binaryClassification = Eigen::VectorXd::Zero(nbOutputs_);

  Eigen::MatrixXd::Index maxIndex;
  classification.maxCoeff(&maxIndex);
  binaryClassification(maxIndex) = 1;

  std::cout << classification.transpose() << std::endl;
  std::cout << binaryClassification.transpose() << std::endl;

  return maxIndex + 1;
}

Eigen::MatrixXd ESN::compute_statematrix(const Eigen::MatrixXd& signal, int nbDataPoints) {
  Eigen::VectorXd totalState = Eigen::VectorXd::Zero(nbInternalUnits_ + nbInputs_ + nbOutputs_);
  Eigen::MatrixXd stateCollection = Eigen::MatrixXd::Zero(nbDataPoints - nbForgetPoints_, nbInternalUnits_ + nbInputs_);
  Eigen::VectorXd internalState(nbInternalUnits_);
  Eigen::VectorXd input(nbInputs_);
  Eigen::VectorXd extendedState(nbInternalUnits_ + nbInputs_);

  int collectData = 0;
  for (int i = 0; i < nbDataPoints; i++) {
    input = inputScaling_.cwiseProduct(signal.row(i).transpose()) + inputShift_;
    totalState.segment(nbInternalUnits_, nbInputs_) = input;

    internalState = plain_esn(totalState);
    extendedState.head(nbInternalUnits_) = internalState;
    extendedState.tail(nbInputs_) = input;

    totalState.head(nbInternalUnits_) = internalState;
    totalState.tail(nbOutputs_) = outputWeights_ * extendedState;

    if (i > nbForgetPoints_ - 1) {
      stateCollection.row(collectData) = extendedState;
      collectData = collectData + 1;
    }
  }
  return stateCollection;
}

Eigen::VectorXd ESN::plain_esn(const Eigen::VectorXd& totalState) {
//  // The following equations are the learning equations
  Eigen::MatrixXd outputFeedback = Eigen::MatrixXd::Zero(nbOutputs_, nbOutputs_);
  outputFeedback.diagonal() = feedbackScaling_;
  outputFeedback = feedbackWeights_ * outputFeedback;
  Eigen::MatrixXd totalWeights = Eigen::MatrixXd::Zero(nbInternalUnits_, nbInternalUnits_ + nbInputs_ + nbOutputs_);
  totalWeights.topLeftCorner(nbInternalUnits_, nbInternalUnits_) = internalWeights_;
  totalWeights.block(0, nbInternalUnits_, nbInternalUnits_, nbInputs_) = inputWeights_;
  totalWeights.topRightCorner(nbInternalUnits_, nbOutputs_) = outputFeedback;
  Eigen::VectorXd internalState = totalWeights * totalState;

  // apply the reservoir activation function
  return internalState.unaryExpr(&tanh);
}

Eigen::VectorXd ESN::s_classify2(const Eigen::MatrixXd& outputSeq, const int& nbSplits) const {
  int split = floor(static_cast<double>(outputSeq.rows()) / nbSplits);
  Eigen::MatrixXd averagePredictedOutput = Eigen::MatrixXd::Zero(nbSplits, nbOutputs_);
  for (std::size_t i = 0; i < nbSplits - 1; ++i) {
    averagePredictedOutput.row(i) = outputSeq.middleRows(i * split, split).colwise().mean();
  }
  averagePredictedOutput.row(nbSplits - 1) =
      outputSeq.bottomRows(outputSeq.rows() - (nbSplits - 1) * split).colwise().mean();
  Eigen::VectorXd sumOfAveragePredictedOutput =
      averagePredictedOutput.colwise().sum() / averagePredictedOutput.colwise().sum().sum();

  return sumOfAveragePredictedOutput;
}

}
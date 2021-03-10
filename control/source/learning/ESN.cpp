#include "learning/ESN.h"

#include <iostream>
#include <fstream>

namespace learning {

ESN::ESN(const std::string& file) {
  std::ifstream esnFile(file);
  if (esnFile.is_open()) {
    esnFile >> nbForgetPoints_;
    esnFile >> nbInternalUnits_;
    esnFile >> nbInputs_;
    esnFile >> nbOutputs_;
    fillMatrix(esnFile, internalWeightsUnitSR_, nbInternalUnits_, nbInternalUnits_);
    esnFile >> nbTotalUnits_;
    fillMatrix(esnFile, inputWeights_, nbInternalUnits_, nbInputs_);
    fillMatrix(esnFile, outputWeights_, nbOutputs_, nbInternalUnits_ + nbInputs_);
    fillMatrix(esnFile, feedbackWeights_, nbInternalUnits_, nbOutputs_);
    fillVector(esnFile, inputScaling_, nbInputs_);
    fillVector(esnFile, inputShift_, nbInputs_);
    fillVector(esnFile, teacherScaling_, nbOutputs_);
    fillVector(esnFile, teacherShift_, nbOutputs_);
    fillVector(esnFile, feedbackScaling_, nbOutputs_);
    fillVector(esnFile, timeConstants_, nbInternalUnits_);
    esnFile >> leakage_;
    fillMatrix(esnFile, internalWeights_, nbInternalUnits_, nbInternalUnits_);
  }
}

void ESN::fillMatrix(std::ifstream& data, Eigen::MatrixXd& matrix, const int& size1, const int& size2) {
  std::vector<double> input(size1 * size2);
  for (std::size_t i = 0; i < size1 * size2; ++i) {
    data >> input.at(i);
  }

  matrix.resize(size1, size2);
  matrix =
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(input.data(), size1, size2);
}

void ESN::fillVector(std::ifstream& data, Eigen::VectorXd& vector, const int& size) {
  std::vector<double> input(size);
  for (std::size_t i = 0; i < size; ++i) {
    data >> input.at(i);
  }

  vector.resize(size);
  vector = Eigen::Map<Eigen::VectorXd>(input.data(), size);
}

void ESN::printAll() {
  std::cout << "number of Internal Units: " << nbInternalUnits_ << std::endl;
  std::cout << "number of Input Units: " << nbInputs_ << std::endl;
  std::cout << "number of Output Units: " << nbOutputs_ << std::endl;

  std::cout << "internal Weights UnitSR" << std::endl;
//  std::cout << internalWeightsUnitSR_ << std::endl;

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

  std::cout << "time Constants:" << std::endl;
//  std::cout << timeConstants_.transpose() << std::endl;

  std::cout << "internal Weights:" << std::endl;
//  std::cout << internalWeights_ << std::endl;

  std::cout << "leakage: " << leakage_ << std::endl;
}

int ESN::test_esn(const Eigen::MatrixXd& data, int nbDataPoints) {
  Eigen::MatrixXd stateCollection = compute_statematrix(data, nbDataPoints);

  Eigen::MatrixXd outputSequence = stateCollection * outputWeights_.transpose();
  outputSequence -= teacherShift_.transpose().replicate(nbDataPoints - nbForgetPoints_, 1);
  outputSequence *= teacherScaling_.asDiagonal().inverse();

//  int classification_result = s_classify(outputSequence, nbDataPoints - nbForgetPoints_);
  Eigen::VectorXd classification = s_classify2(outputSequence, 1);

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
    averagePredictedOutput.row(i) = outputSeq.topRows(split).rowwise().mean();
  }
  averagePredictedOutput.row(nbSplits - 1) =
      outputSeq.middleRows((nbSplits - 1) * split, outputSeq.rows()).colwise().mean();
  Eigen::VectorXd sumOfAveragePredictedOutput = averagePredictedOutput.colwise().sum().normalized();

  return sumOfAveragePredictedOutput;
}

}
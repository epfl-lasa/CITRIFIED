#pragma once

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>

namespace learning {

class ESN {
public:
  explicit ESN(const std::string& file);

  ~ESN() = default;

  void printAll();

  Eigen::MatrixXd compute_statematrix(const Eigen::MatrixXd& data, int nbDataPoints);

//  int s_classify(std::vector<std::vector<double>> outputSeq, int npoints);
//  int s_classify(Eigen::MatrixXd outputSeq, int npoints);
  Eigen::VectorXd s_classify2(const Eigen::MatrixXd& outputSeq, const int& nbSplits = 1) const;

  Eigen::VectorXd plain_esn(const Eigen::VectorXd& tState);

  int test_esn(const Eigen::MatrixXd& data, int nbDataPoints);

private:
  void fillMatrix(std::ifstream& data, Eigen::MatrixXd& matrix, const int& size1, const int& size2);
  void fillVector(std::ifstream& data, Eigen::VectorXd& vector, const int& size);

  int nbForgetPoints_;
  int nbInternalUnits_;
  int nbInputs_;
  int nbOutputs_;

  Eigen::MatrixXd internalWeightsUnitSR_; // nbInternalUnits_ x nbInternalUnits
  int nbTotalUnits_;

  Eigen::MatrixXd inputWeights_; // size nbInternalUnits_ x nbInputs_
  Eigen::MatrixXd outputWeights_; // nbOutputs_ x col_outWeights_
  Eigen::MatrixXd feedbackWeights_; // nbInternalUnits_ x nbOutputs_
  Eigen::VectorXd inputScaling_; // nbInputs_
  Eigen::VectorXd inputShift_; // nbInputs_
  Eigen::VectorXd teacherScaling_; // nbOutputs_
  Eigen::VectorXd teacherShift_; // nbOutputs_
  Eigen::VectorXd feedbackScaling_; // nbOutputs_
  float leakage_;
  Eigen::VectorXd timeConstants_; // nbInternalUnits_
  Eigen::MatrixXd internalWeights_; // nbInternalUnits_ x nbInternalUnits_
};
}
#pragma once

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace learning {

class ESN {
public:
  explicit ESN(const std::string& file);

  ~ESN() = default;

  void printAll();

  Eigen::MatrixXd compute_statematrix(const Eigen::MatrixXd& data, int nbDataPoints);

  Eigen::VectorXd s_classify2(const Eigen::MatrixXd& outputSeq, const int& nbSplits = 1) const;

  Eigen::VectorXd plain_esn(const Eigen::VectorXd& tState);

  int test_esn(const Eigen::MatrixXd& data, int nbDataPoints);

private:
  static void readMatrix(const YAML::Node& params, const std::string& paramName, Eigen::MatrixXd& matrix,
                         const int& rows, const int& cols);
  static void readVector(const YAML::Node& params, const std::string& paramName, Eigen::VectorXd& matrix,
                         const int& rows);

  int nbForgetPoints_;
  int nbInternalUnits_;
  int nbInputs_;
  int nbOutputs_;
  int nbTotalUnits_;

  Eigen::MatrixXd internalWeights_; // nbInternalUnits_ x nbInternalUnits_
  Eigen::MatrixXd inputWeights_; // nbInternalUnits_ x nbInputs_
  Eigen::MatrixXd outputWeights_; // nbOutputs_ x col_outWeights_
  Eigen::MatrixXd feedbackWeights_; // nbInternalUnits_ x nbOutputs_

  Eigen::VectorXd inputScaling_; // nbInputs_
  Eigen::VectorXd inputShift_; // nbInputs_
  Eigen::VectorXd teacherScaling_; // nbOutputs_
  Eigen::VectorXd teacherShift_; // nbOutputs_
  Eigen::VectorXd feedbackScaling_; // nbOutputs_
};
}
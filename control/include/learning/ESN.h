#pragma once

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace learning {

struct esnPrediction {
  std::string className;
  int classIndex;
  Eigen::VectorXd predictions;
};

class ESN {
public:
  explicit ESN(const std::string& file);

  ~ESN() = default;

  void printAll();

  esnPrediction predict(const Eigen::MatrixXd& data, const int& nbPredictionSplits = 1);

  int inputDimensions() const;

private:
  static void readInt(const YAML::Node& params, const std::string& paramName, int& var);

  template <typename T>
  static void readMatrix(const YAML::Node& params, const std::string& paramName, T& matrix, const int& rows,
                         const int& cols = 1);

  Eigen::MatrixXd collectStates(const Eigen::MatrixXd& data);

  esnPrediction classify(const Eigen::MatrixXd& outputSeq, const int& nbSplits) const;

  int nbForgetPoints_;
  int nbInternalUnits_;
  int nbInputs_;
  int nbOutputs_;
  std::vector<std::string> classNames_;

  Eigen::MatrixXd outputWeights_; // nbOutputs_ x (nbInternalUnits + nbInputs)

  Eigen::MatrixXd systemEquationWeightMatrix_; // nbInternalUnits_ x (nbInternalUnits + nbInputs + nbOutputs)
  Eigen::MatrixXd invTeacherScalingMatrix_; // nbOutputs_ x nbOutputs_

  Eigen::VectorXd inputScaling_; // nbInputs_
  Eigen::VectorXd inputShift_; // nbInputs_
  Eigen::VectorXd teacherScaling_; // nbOutputs_
  Eigen::VectorXd teacherShift_; // nbOutputs_
  Eigen::VectorXd feedbackScaling_; // nbOutputs_
};
}
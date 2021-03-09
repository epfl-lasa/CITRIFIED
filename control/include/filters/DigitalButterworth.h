#pragma once

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>

namespace filter {

class DigitalButterworth {
public:
  explicit DigitalButterworth(const std::string& filterName, const std::string& filePath, const int& dimensions);

  double computeFilterOutput(const double& input);

  Eigen::VectorXd computeFilterOutput(const Eigen::VectorXd& input);

  void resetFilter();

private:
  double differenceEquations(const double& input, const int& wIndex);

  const int dim_;
  int order_;
  std::vector<double> numeratorCoeffs_;
  std::vector<double> denominatorCoeffs_;
  std::vector<Eigen::VectorXd> w_;
};

}
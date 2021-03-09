#pragma once

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>

namespace filter {

class DigitalButterworth {
public:
  explicit DigitalButterworth(const std::string& filterName, const std::string& filePath, const int& dimensions);

  double computeFilterOutput(double input);
  Eigen::VectorXd computeFilterOutput(const Eigen::VectorXd& input);

  void resetFilter();

private:
  Eigen::VectorXd differenceEquations(const Eigen::VectorXd& input);

  const int dim_;
  int order_;
  std::vector<double> numeratorCoeffs_;
  std::vector<double> denominatorCoeffs_;
  std::vector<Eigen::VectorXd> w_;
};

}
#pragma once

#include <string>
#include <vector>

namespace filter {

class DigitalButterFilter {
public:
  explicit DigitalButterFilter(const std::string& filterName, const std::string& filePath);

  double computeFilterOutput(const double& input);

  void resetFilter();

private:
  double order_;
  double output_ = 0;
  double gain_;
  std::vector<double> numeratorCoeffs_;
  std::vector<double> denominatorCoeffs_;
  std::vector<double> w_;
};

}
#pragma once

#include <string>
#include <vector>

namespace filter {

class DigitalButterworth {
public:
  explicit DigitalButterworth(const std::string& filterName, const std::string& filePath);

  double computeFilterOutput(const double& input);

  void resetFilter();

private:
  double order_;
  std::vector<double> numeratorCoeffs_;
  std::vector<double> denominatorCoeffs_;
  std::vector<double> w_;
};

}
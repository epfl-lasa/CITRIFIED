#include "filters/DigitalButterworth.h"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace filter {

DigitalButterworth::DigitalButterworth(const std::string& filterName, const std::string& filePath,
                                       const int& dimensions) : dim_(dimensions) {
  YAML::Node params = YAML::LoadFile(filePath);
  order_ = params[filterName]["order"].as<int>();
  auto numCoeffs = params[filterName]["numerator_coefficients"].as<std::vector<double>>();
  auto denomCoeffs = params[filterName]["denominator_coefficients"].as<std::vector<double>>();
  if (numCoeffs.size() != order_ + 1 || denomCoeffs.size() != order_ + 1) {
    throw std::invalid_argument("The parameters for your filter " + filterName
                                    + " are incorrect. Check the size of the numerator and denominator vectors.");
  }
  numeratorCoeffs_ = numCoeffs;
  denominatorCoeffs_ = denomCoeffs;
  w_.resize(order_, Eigen::VectorXd::Zero(dim_));
  resetFilter();
}

double DigitalButterworth::computeFilterOutput(double input) {
  if (dim_ != 1) {
    throw std::invalid_argument(
        "This filter has input dimension " + std::to_string(dim_) + ". Check the size of your input");
  }
  Eigen::VectorXd vectorInput(1);
  vectorInput << input;
  Eigen::VectorXd output = differenceEquations(vectorInput);
  return output(0);
}

Eigen::VectorXd DigitalButterworth::computeFilterOutput(const Eigen::VectorXd& input) {
  if (dim_ != input.size()) {
    throw std::invalid_argument(
        "This filter has input dimension " + std::to_string(dim_) + ". Check the size of your input");
  }

  return differenceEquations(input);
}

Eigen::VectorXd DigitalButterworth::differenceEquations(const Eigen::VectorXd& input) {
  Eigen::VectorXd output = numeratorCoeffs_.at(0) * input + w_.at(0);
  for (size_t i = 0; i < order_ - 1; ++i) {
    w_.at(i) = numeratorCoeffs_.at(i + 1) * input + w_.at(i + 1) - denominatorCoeffs_.at(i + 1) * output;
  }
  w_.at(order_ - 1) = numeratorCoeffs_.back() * input - denominatorCoeffs_.back() * output;
  return output;
}

void DigitalButterworth::resetFilter() {
  std::fill(w_.begin(), w_.end(), Eigen::VectorXd::Zero(dim_));
}

}
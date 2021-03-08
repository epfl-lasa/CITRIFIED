#include "filters/DigitalButterFilter.h"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace filter {

DigitalButterFilter::DigitalButterFilter(const std::string& filterName, const std::string& filePath) {
  YAML::Node params = YAML::LoadFile(filePath);
  order_ = params[filterName]["order"].as<double>();
  auto numCoeffs = params[filterName]["numerator_coefficients"].as<std::vector<double>>();
  auto denomCoeffs = params[filterName]["denominator_coefficients"].as<std::vector<double>>();
  if (numCoeffs.size() != order_ + 1 || denomCoeffs.size() != order_ + 1) {
    throw std::invalid_argument("The parameters for your filter " + filterName
                                    + " are incorrect. Check the size of the numerator and denominator vectors.");
  }
  numeratorCoeffs_ = numCoeffs;
  denominatorCoeffs_ = denomCoeffs;
  w_.resize(order_);
}

double DigitalButterFilter::computeFilterOutput(const double& input) {
  output_ = numeratorCoeffs_.at(0) * input + w_.at(0);
  for (size_t i = 0; i < order_ - 1; ++i) {
    w_.at(i) = numeratorCoeffs_.at(i + 1) * input + w_.at(i + 1) - denominatorCoeffs_.at(i + 1) * output_;
  }
  w_.back() = numeratorCoeffs_.back() * input - denominatorCoeffs_.back() * output_;
  return output_;
}

void DigitalButterFilter::resetFilter() {
  for (size_t i = 0; i < order_; ++i) {
    w_.at(i) = 0;
  }
  output_ = 0;
}

}
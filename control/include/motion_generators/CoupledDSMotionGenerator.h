#pragma once

#include <memory>
#include <fast_gmm/GMRDynamics.h>
#include <eigen3/Eigen/Core>

namespace motion_generator {

class CoupledDSMotionGenerator {
public:
  CoupledDSMotionGenerator(double frequency,
                           int k_gmm,
                           int dim,
                           std::vector<double> priors,
                           std::vector<double> mu,
                           std::vector<double> sigma,
                           double muScale,
                           double sigmaScale,
                           std::vector<double> attractor);

  ~CoupledDSMotionGenerator() = default;

  bool init();

  Eigen::VectorXd computeDesiredVelocity(const Eigen::VectorXd& input);

private:

  bool initializeDS();

  int k_gmm_;
  int dim_;
  double dt_;
  std::vector<double> priors_;
  std::vector<double> mu_;
  std::vector<double> sigma_;
  std::vector<double> attractor_;
  double muScale_;
  double sigmaScale_;
  std::unique_ptr<GMRDynamics> SED_GMM_;

};

}
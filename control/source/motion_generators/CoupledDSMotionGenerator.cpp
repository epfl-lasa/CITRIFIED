#include "motion_generators/CoupledDSMotionGenerator.h"

namespace motion_generator {

CoupledDSMotionGenerator::CoupledDSMotionGenerator(double frequency,
                                                   int k_gmm,
                                                   int dim,
                                                   std::vector<double> priors,
                                                   std::vector<double> mu,
                                                   std::vector<double> sigma,
                                                   double muScale,
                                                   double sigmaScale,
                                                   std::vector<double> attractor) :
    dt_(1 / frequency),
    k_gmm_(k_gmm),
    dim_(dim),
    priors_(std::move(priors)),
    mu_(std::move(mu)),
    sigma_(std::move(sigma)),
    muScale_(muScale),
    sigmaScale_(sigmaScale),
    attractor_(std::move(attractor)) {}

bool CoupledDSMotionGenerator::init() {
  if (!initializeDS()) {
    std::cerr << "ERROR initializing the DS" << std::endl;
    return false;
  } else {
    return true;
  }
}

bool CoupledDSMotionGenerator::initializeDS() {
  if (priors_.size() != k_gmm_) {
    std::cerr << "initializeDS: " << k_gmm_ << " priors are expected while " << priors_.size() << " are provided."
              << std::endl;
    return false;
  }

  if (mu_.size() != k_gmm_ * dim_) {
    std::cerr << "InitializeDS: " << k_gmm_ * dim_ << " elements in Mu are expected while " << mu_.size()
              << " are provided." << std::endl;
    return false;
  }

  if (sigma_.size() != k_gmm_ * dim_ * dim_) {
    std::cerr << "InitializeDS: " << k_gmm_ * dim_ * dim_ << " elements in sigma are expected while " << sigma_.size()
              << " are provided." << std::endl;
    return false;
  }

  if (attractor_.size() != 7) {
    std::cerr
        << "InitializeDS: Please provide 7 (3 for position xyz and 4 for quaternion wxyz) elements for the attractor. It has "
        << attractor_.size() << " elements." << std::endl;
    return false;
  }

  /* Scale Mu and Sigma given scaling factor, default1=, but some models can have really high numbers */
  for (int i = 0; i < k_gmm_ * dim_; i++) {
    mu_[i] = mu_[i] * muScale_;
  }

  for (int i = 0; i < k_gmm_ * dim_ * dim_; i++) {
    sigma_[i] = sigma_[i] * sigmaScale_;
  }

  SED_GMM_ = std::make_unique<GMRDynamics>(k_gmm_, dim_, dt_, priors_, mu_, sigma_);
  SED_GMM_->initGMR(0, 2, 3, 5); //get the input and output Mu and sigma

  return true;
}

Eigen::VectorXd CoupledDSMotionGenerator::computeDesiredVelocity(const Eigen::VectorXd& input) {
  MathLib::Vector pose(k_gmm_);
  for (int i = 0; i < k_gmm_; ++i) {
    pose[i] = input(i) - attractor_[i];
  }

  MathLib::Vector desiredVelocity = SED_GMM_->getVelocity(pose);
  Eigen::Vector3d desiredVelocityEigen(desiredVelocity(0), desiredVelocity(1), desiredVelocity(2));
  Eigen::VectorXd desiredTwist = Eigen::VectorXd(k_gmm_);

  for (int i = 0; i < k_gmm_; ++i) {
    desiredTwist[i] = desiredVelocity(i);
  }

  return desiredTwist;

}
}
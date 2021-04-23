//
// Created by Rui Wu on 2021/4/16.
//
#include "motion_generators/coupledDSMotionGenerator.h"
#include "state_representation/space/cartesian/CartesianState.hpp"

using namespace state_representation;

//namespace motion_generator {
coupledDSMotionGenerator::coupledDSMotionGenerator(
        double frequency,
        //-----SEDS
        int K_gmm,
        int dim,
        std::vector<double> Priors,
        std::vector<double> Mu,
        std::vector<double> Sigma,
        double Mu_scale,
        double Sigma_scale,
        std::vector<double> attractor)
        :
//-----SEDS
        K_gmm_(K_gmm),
        dim_(dim),
        Priors_(Priors),
        Mu_(Mu),
        Sigma_(Sigma),
        Mu_scale_(Mu_scale),
        Sigma_scale_(Sigma_scale),
        attractor_(attractor){
}

coupledDSMotionGenerator *coupledDSMotionGenerator::me = NULL;

bool coupledDSMotionGenerator::Init() {

  me = this;
  _stop = false;

  if (!InitializeDS()) {
    std::cerr << "ERROR intializing the DS" << std::endl;
    return false;
  }

  return true;
}

bool coupledDSMotionGenerator::InitializeDS() {

//-----------for SEDS-------------
  if (Priors_.size() != K_gmm_) {
    std::cerr << "InitializeDS: " << K_gmm_ << " priors is expected while " << Priors_.size() << " is provided."
              << std::endl;
    return false;
  }

  if (Mu_.size() != K_gmm_ * dim_) {
    std::cerr <<
              "InitializeDS: " << K_gmm_ * dim_ << " elements in Mu is expected while " << Mu_.size()
              << " is provided." << std::endl;
    return false;
  }

  if (Sigma_.size() != K_gmm_ * dim_ * dim_) {
    std::cerr <<
              "InitializeDS: " << K_gmm_ * dim_ * dim_ << " elements in Sigma is expected while " << Sigma_.size()
              << " is provided." << std::endl;
    return false;
  }

  if (attractor_.size() != 6) {
    std::cerr <<
              "InitializeDS: Please provide 6 elements for the attractor. It has " << attractor_.size()
              << " elements." << std::endl;
    return false;
  }

  /* Scale Mu and Sigma given scaling factor, default1=, but some models can have really high numbers */
  for (int i = 0; i < K_gmm_ * dim_; i++) {
    Mu_[i] = Mu_[i] * Mu_scale_;
  }

  for (int i = 0; i < K_gmm_ * dim_ * dim_; i++) {
    Sigma_[i] = Sigma_[i] * Sigma_scale_;
  }

  SED_GMM_.reset(new GMRDynamics(K_gmm_, dim_, dt_, Priors_, Mu_, Sigma_));
  SED_GMM_->initGMR(0, 2, 3, 5); //get the input and output Mu and sigma

  // initializing the filter
  CCDyn_filter_.reset(new CDDynamics(3, dt_, Wn_));
  CCDyn_filter_1_.reset(new CDDynamics(3, dt_, Wn_));
  // CCDyn_filter_2_.reset (new CDDynamics(3, dt_, Wn_));

  // we should set the size automagically
  velLimits_.Resize(3);
  CCDyn_filter_->SetVelocityLimits(velLimits_);
  CCDyn_filter_1_->SetVelocityLimits(velLimits_);
  // CCDyn_filter_2_->SetVelocityLimits(velLimits_);

  accLimits_.Resize(3);    /* Set the desired orientation as the initial one */
//  qx = msg_real_pose_.orientation.x;
//  qy = msg_real_pose_.orientation.y;
//  qz = msg_real_pose_.orientation.z;
//  qw = msg_real_pose_.orientation.w;
  CCDyn_filter_->SetAccelLimits(velLimits_);
  CCDyn_filter_1_->SetAccelLimits(velLimits_);
  // CCDyn_filter_2_->SetAccelLimits(accLimits_);

  MathLib::Vector initial(3);

  initial.Zero();

  CCDyn_filter_->SetState(initial);
  CCDyn_filter_->SetTarget(initial);

  CCDyn_filter_1_->SetState(initial);
  CCDyn_filter_1_->SetTarget(initial);

  // CCDyn_filter_2_->SetState(initial);
  // CCDyn_filter_2_->SetTarget(initial);
  return true;
}
//}

MathLib::Vector coupledDSMotionGenerator::ComputeDesiredVelocity(const CartesianState& eeInRobot_) {
  Trans_pose.Resize(3);
  Trans_pose(0)=eeInRobot_.get_position().x();
  Trans_pose(1)=eeInRobot_.get_position().y();
  Trans_pose(2)=eeInRobot_.get_position().z();

  desired_velocity_ = SED_GMM_->getVelocity(Trans_pose);
  return desired_velocity_;

}

coupledDSMotionGenerator::~coupledDSMotionGenerator(){

}
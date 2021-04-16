//
// Created by wr on 2021/4/13.
//

//#include <yaml-cpp/yaml.h>
//#include <dynamical_systems/Linear.hpp>
//#include <dynamical_systems/Ring.hpp>
//#include "controllers/TwistController.h"
//#include "sensors/ForceTorqueSensor.h"
//#include "franka_lwi/franka_lwi_utils.h"
//#include "sensors/RigidBodyTracker.h"
//#include "filters/DigitalButterworth.h"
//#include "network/interfaces.h"
//#include "logger/JSONLogger.h"
//#include "learning/ESNWrapper.h"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "motion_generators/coupledDSMotionGenerator.h"

using namespace state_representation;

bool coupledDSMotionGenerator::InitializeDS() {

//-----------for SEDS-------------
  if (Priors_.size() != K_gmm_) {
    ROS_ERROR_STREAM("InitializeDS: " << K_gmm_ << " priors is expected while " << Priors_.size() << " is provided.");
    return false;
  }

  if (Mu_.size() != K_gmm_ * dim_) {
    ROS_ERROR_STREAM(
            "InitializeDS: " << K_gmm_ * dim_ << " elements in Mu is expected while " << Mu_.size() << " is provided.");
    return false;
  }

  if (Sigma_.size() != K_gmm_ * dim_ * dim_) {
    ROS_ERROR_STREAM(
            "InitializeDS: " << K_gmm_ * dim_ * dim_ << " elements in Sigma is expected while " << Sigma_.size()
                             << " is provided.");
    return false;
  }

  if (attractor_.size() != 6) {
    ROS_ERROR_STREAM(
            "InitializeDS: Please provide 6 elements for the attractor. It has " << attractor_.size() << " elements.");
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
}

int main(int, char**){
  double x=0.1,y=0.1,z=0.1;
  CartesianState eeInRobot = CartesianState::Random("ee","robot");
  eeInRobot.set_position(x,y,z);
  std::cerr<<"eeInRobot: "<<eeInRobot<<std::endl;
  return 0;
}

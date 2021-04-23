//
// Created by wr on 2021/4/13.
//

#include <yaml-cpp/yaml.h>
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

int main(int, char**){

  double frequency=256,Mu_scale=1, Sigma_scale=1;
//----- SEDS
  std::string filepath = std::string(TRIAL_CONFIGURATION_DIR) + "SEDS_parameters.yaml";
  YAML::Node params = YAML::LoadFile(filepath);
  auto K_gmm = params["K"].as<int>();
  auto dim = params["dim"].as<int>();
  auto Priors = params["Priors"].as<std::vector<double>>();
  auto Mu = params["Mu"].as<std::vector<double>>();
  auto Sigma = params["Sigma"].as<std::vector<double>>();
  auto attractor = params["attractor"].as<std::vector<double>>();

  double x=0.1,y=0.1,z=2;
  CartesianState eeInRobot = CartesianState::Random("ee","robot");
//  std::cerr<<"eeInRobot: "<<eeInRobot<<std::endl;

  MathLib::Vector desired_velocity;

  coupledDSMotionGenerator coupledDSMotionGenerator(frequency,
          //----- SEDS
                                             K_gmm, dim, Priors, Mu, Sigma,
                                             Mu_scale, Sigma_scale,
                                             attractor);

  if (!coupledDSMotionGenerator.Init()) {
    return -1;
  }
  else {
    while (z>0){
      z=z-0.01;
      eeInRobot.set_position(x,y,z);
      desired_velocity = coupledDSMotionGenerator.ComputeDesiredVelocity(eeInRobot);
      std::cerr<<"desired_velocity"<<desired_velocity<<std::endl;
    }

  };



  return 0;
}

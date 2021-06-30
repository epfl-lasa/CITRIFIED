#include <yaml-cpp/yaml.h>
#include <state_representation/space/cartesian/CartesianState.hpp>

#include <dynamical_systems/Linear.hpp>
#include "motion_generators/CoupledDSMotionGenerator.h"
#include "motion_generators/LagsDSMotionGenerator.h"

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "controllers/impedance/CartesianTwistController.hpp"
#include "franka_lwi/franka_lwi_utils.h"
#include "network/interfaces.h"

using namespace state_representation;

//int main(int, char**) {
//
//  double frequency = 256, muScale = 1, sigmaScale = 1;
//
//  // load SEDS
//  std::string filepath = std::string(TRIAL_CONFIGURATION_DIR) + "SEDS_parameters.yaml";
//  YAML::Node params = YAML::LoadFile(filepath);
//  auto k_gmm = params["K"].as<int>();
//  auto dim = params["dim"].as<int>();
//  auto priors = params["Priors"].as<std::vector<double>>();
//  auto mu = params["Mu"].as<std::vector<double>>();
//  auto sigma = params["Sigma"].as<std::vector<double>>();
//  auto attractor = params["attractor"].as<std::vector<double>>();
//
//  double x = 0.1, y = 0.1, z = 2;
//  CartesianPose eeInRobot = CartesianPose::Random("ee", "robot");
//
//  motion_generator::CoupledDSMotionGenerator ds(frequency, k_gmm, dim, priors, mu, sigma, muScale, sigmaScale, attractor);
//
//  if (!ds.init()) {
//    return -1;
//  }
//
//  while (z > 0) {
//  z = z - 0.01;
//  eeInRobot.set_position(x, y, z);
//  auto twist = ds.computeDesiredVelocity(eeInRobot);
//  std::cerr << "desired twist" << twist << std::endl;
//  }
//
//  return 0;
//}

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  state_representation::CartesianState robot("end-effector", "franka");
  state_representation::Jacobian jacobian("franka", 7, "end-effector", "franka");

  controllers::impedance::CartesianTwistController ctrl(100, 100, 5, 5);

  network::Interface franka(network::InterfaceType::FRANKA_QUEBEC_17);
  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  double frequency = 900, muScale = 1, sigmaScale = 1;
  // load SEDS
//  std::string filepath = std::string(TRIAL_CONFIGURATION_DIR) + "SEDS_parameters.yaml";
//  YAML::Node params = YAML::LoadFile(filepath);
//  auto k_gmm = params["K"].as<int>();
//  auto dim = params["dim"].as<int>();
//  auto priors = params["Priors"].as<std::vector<double>>();
//  auto mu = params["Mu"].as<std::vector<double>>();
//  auto sigma = params["Sigma"].as<std::vector<double>>();
//  auto attractor_vec = params["attractor"].as<std::vector<double>>();
//
//  motion_generator::CoupledDSMotionGenerator
//      ds(frequency, k_gmm, dim, priors, mu, sigma, muScale, sigmaScale, attractor_vec);

  // set LAGS DS
  std::string filepath = std::string(TRIAL_CUT_CURVE_CONFIGURATION_DIR) + "cut_curve_low_gain_coupled.yml";
  YAML::Node LAGS_params = YAML::LoadFile(filepath);
  auto K_gmm = LAGS_params["K"].as<int>();
  auto M_gmm = LAGS_params["M"].as<int>();
  auto Priors = LAGS_params["Priors"].as<std::vector<double>>();
  auto Mu = LAGS_params["Mu"].as<std::vector<double>>();
  auto Sigma = LAGS_params["Sigma"].as<std::vector<double>>();
  auto A_g = LAGS_params["A_g"].as<std::vector<double>>();
  auto att_g = LAGS_params["att_g"].as<std::vector<double>>();
  auto A_l = LAGS_params["A_l"].as<std::vector<double>>();
  auto A_d = LAGS_params["A_d"].as<std::vector<double>>();
  auto att_l = LAGS_params["att_l"].as<std::vector<double>>();
  auto w_l = LAGS_params["w_l"].as<std::vector<double>>();
  auto b_l = LAGS_params["b_l"].as<std::vector<double>>();
  auto scale = LAGS_params["scale"].as<double>();
  auto b_g = LAGS_params["b_g"].as<double>();
  auto gpr_path = std::string(TRIAL_CUT_CURVE_CONFIGURATION_DIR) + "GPR_model.txt";

  motion_generator::LagsDSMotionGenerator
      ds(frequency, K_gmm, M_gmm, Priors, Mu, Sigma, A_g, att_g, A_l, A_d, att_l, w_l, b_l, scale, b_g, gpr_path);


  if (!ds.init()) {
    return -1;
  }

  state_representation::CartesianPose attractor = state_representation::CartesianPose("attractor",
                                                                                      Eigen::Quaterniond(0, 1, 0, 0),
                                                                                      "franka");
  std::vector<double> gains = {0.0, 0.0, 0.0, 10.0, 10.0, 10.0};
  dynamical_systems::Linear<state_representation::CartesianState> orientationDS(attractor, gains);

  // control loop
  while (franka.receive(state)) {
    frankalwi::utils::toCartesianState(state, robot);
    frankalwi::utils::toJacobian(state.jacobian, jacobian);

    state_representation::CartesianTwist dsTwist = orientationDS.evaluate(robot);
//    auto velocity = ds.computeDesiredVelocity(robot.get_position());
    auto velocity = ds.computeDesiredVelocity(robot.get_position().head(2));
    velocity = Eigen::Vector3d(velocity(0), velocity(1), 0);
    dsTwist.set_linear_velocity(velocity);
    std::cout << dsTwist << std::endl;
    dsTwist.clamp(0.2, 0.5);

    state_representation::JointTorques joint_command = ctrl.compute_command(dsTwist, robot, jacobian);

    frankalwi::utils::fromJointTorque(joint_command, command);
    franka.send(command);
  }
}
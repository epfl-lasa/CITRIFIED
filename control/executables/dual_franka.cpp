#include <iostream>

#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/JointTorques.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <dynamical_systems/Linear.hpp>

#include "controllers/FrankaController.h"
#include "controllers/impedance/CartesianTwistController.hpp"

using namespace state_representation;
using namespace dynamical_systems;

class DualFrankaController {
public:
  DualFrankaController() :
    franka_papa(network::InterfaceType::FRANKA_PAPA_16, "papa", "ee_papa"),
    franka_quebec(network::InterfaceType::FRANKA_QUEBEC_17, "quebec", "ee_quebec"),
    ctrl_papa(0, 0, 0, 0),
    ctrl_quebec(100, 100, 5, 5) {

    // assume frame papa = world
    frame_papa = CartesianState::Identity("papa");
    frame_quebec = CartesianState("quebec");
    frame_quebec.set_position(0.899, 0, 0);
    frame_quebec.set_orientation(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())));

    std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};

    state_representation::CartesianPose attractor_papa("attractor_papa",
                                                       Eigen::Vector3d(0.35, 0, 0.5),
                                                       frame_papa.get_name());
    attractor_papa.set_position(0.35, 0, 0.5);
    ds_papa = Linear<CartesianState>(attractor_papa, gains);

    state_representation::CartesianPose attractor_quebec("attractor_quebec",
                                                         Eigen::Vector3d(0.35, 0, 0.5),
                                                         frame_quebec.get_name());
    ds_quebec = Linear<CartesianState>(attractor_quebec, gains);

    franka_papa.set_callback([this] (const CartesianState& state, const Jacobian& jacobian) -> JointTorques {
      return control_loop_papa(state, jacobian);
    });

    franka_quebec.set_callback([this] (const CartesianState& state, const Jacobian& jacobian) -> JointTorques {
      return control_loop_quebec(state, jacobian);
    });
  }

  JointTorques control_loop_papa(const CartesianState& state, const Jacobian& jacobian) {
    CartesianTwist dsTwist = ds_papa.evaluate(state);
    dsTwist.clamp(0.5, 0.75);

    return ctrl_papa.compute_command(dsTwist, state, jacobian);
  }

  JointTorques control_loop_quebec(const CartesianState& state, const Jacobian& jacobian) {
    auto papa_ee_state = franka_papa.get_state();
    if (!papa_ee_state.is_empty()) {
      auto target_in_papa_ee = CartesianPose("attractor",
                                             Eigen::Vector3d(0, 0, 0.2),
                                             Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 1, 0))),
                                             papa_ee_state.get_name());
      ds_quebec.set_attractor(frame_quebec.inverse() * frame_papa * papa_ee_state * target_in_papa_ee);
    } else {
      ds_quebec.set_attractor(state);
    }

    CartesianTwist dsTwist = ds_quebec.evaluate(state);
    dsTwist.clamp(0.5, 0.75);

    return ctrl_quebec.compute_command(dsTwist, state, jacobian);
  }

  controllers::FrankaController franka_papa;
  controllers::FrankaController franka_quebec;
  CartesianState frame_papa;
  CartesianState frame_quebec;
  Linear<CartesianState> ds_papa;
  Linear<CartesianState> ds_quebec;
  controllers::impedance::CartesianTwistController ctrl_papa;
  controllers::impedance::CartesianTwistController ctrl_quebec;
};

int main(int, char**) {
  std::cout << std::fixed << std::setprecision(3);

  DualFrankaController DFC;

  std::cout << "Starting" << std::endl;
  DFC.franka_papa.start();
  DFC.franka_quebec.start();

  std::string tmp;
  std::cin >> tmp;

  std::cout << "Stopping" << std::endl;
  DFC.franka_papa.stop();
  DFC.franka_quebec.stop();

  std::cout << "Done" << std::endl;
}
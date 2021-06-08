#include <vector>
#include <iostream>

#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <dynamical_systems/Linear.hpp>
#include <dynamical_systems/Circular.hpp>
#include <controllers/impedance/CartesianTwistController.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "franka_lwi/franka_lwi_utils.h"
#include "network/interfaces.h"

class BlendDS {
public:
  BlendDS() {
    orientationDS.set_attractor(state_representation::CartesianPose("world", center, defaultOrientation));
    orientationDS.set_gain(pointGains);

    flatCircleDS = dynamical_systems::Circular(state_representation::CartesianPose("world", center, defaultOrientation),
                                               radius,
                                               1.0,
                                               radialVelocity);
    flatCircleDS.set_planar_gain(circGains[0]);
    flatCircleDS.set_normal_gain(circGains[1]);

    inclinedCircleDS = dynamical_systems::Circular(state_representation::CartesianPose("world",
                                                                                       center,
                                                                                       inclination
                                                                                           * defaultOrientation),
                                                   radius,
                                                   1.0,
                                                   radialVelocity);
    inclinedCircleDS.set_planar_gain(circGains[0]);
    inclinedCircleDS.set_normal_gain(circGains[1]);
  }

  Eigen::Vector3d center = {0.3, 0.3, 0.555};
  Eigen::Quaterniond defaultOrientation = {0, 0.707, 0.707, 0}; //{0.0, 0.919, 0.393, 0.0};
  Eigen::Quaterniond inclination = {0.866, 0.0, 0.5, 0.0};
  double radius = 0.05;
  double radialVelocity = 1.0;
  double circGains[2] = {1000.0, 5.0};
  std::vector<double> pointGains = {0.0, 0.0, 0.0, 10.0, 10.0, 10.0};

  dynamical_systems::Linear<state_representation::CartesianState> orientationDS;
  dynamical_systems::Circular flatCircleDS;
  dynamical_systems::Circular inclinedCircleDS;

  state_representation::CartesianTwist blend(frankalwi::proto::StateMessage<7> state) {
    updateOrientationTarget(state);
    state_representation::CartesianPose pose(state_representation::CartesianPose::Identity("world"));
    frankalwi::utils::toCartesianPose(state, pose);
    auto v1 = orientationDS.evaluate(pose);
    auto v2 = blendCircles(state);

    auto twist = state_representation::CartesianTwist("end-effector",
                                                      v2.get_linear_velocity(),
                                                      v1.get_angular_velocity(),
                                                      "robot");

    return twist.clamped(0.25, 0.75, 1e-3, 1e-3);
  }

private:
  void updateOrientationTarget(frankalwi::proto::StateMessage<7> state) {
    Eigen::Vector3d d = Eigen::Vector3d(state.eePose.position.x, state.eePose.position.y, center.z()) - center;
    double angle;

    angle = atan2(d.y(), d.x());
    if (d.x() < 0) {
      angle = M_PI - angle;
    }

    if (angle > M_PI) {
      angle -= 2 * M_PI;
    }

    // rotate default quaternion about world Z by angle to follow cut in one semicircle,
    // then the opposite way during the restoration phase
    Eigen::Quaterniond
        target = Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())) * defaultOrientation;

    orientationDS.set_attractor(state_representation::CartesianPose("world", center, target));
  }

  state_representation::CartesianTwist blendCircles(frankalwi::proto::StateMessage<7> state) {
    const double xBlendWidth = 0.1 * radius;
    const double heightCutoffStart = 2 * radius;
    const double heightCutoffEnd = 3 * radius;

    double zVel = 0;

    state_representation::CartesianPose pose(state_representation::CartesianPose::Identity("world"));
    frankalwi::utils::toCartesianPose(state, pose);
    auto flatCircleTwist = flatCircleDS.evaluate(pose);
    auto inclinedCircleTwist = inclinedCircleDS.evaluate(pose);

    Eigen::Vector3d l;
    if (state.eePose.position.x - center.x() > xBlendWidth) {
      l = flatCircleTwist.get_linear_velocity();
      zVel = -0.05;
    } else if (state.eePose.position.x - center.x() < -xBlendWidth) {
      l = inclinedCircleTwist.get_linear_velocity();
    } else {
      double scale = (state.eePose.position.x - center.x() + xBlendWidth) / (2 * xBlendWidth);
      l = flatCircleTwist.get_linear_velocity() * scale + inclinedCircleTwist.get_linear_velocity() * (1 - scale);
    }

//    if (state.eePose.position.z - center.z() > heightCutoffStart) {
//      double scale = heightCutoffEnd - (state.eePose.position.z - center.z()) / (heightCutoffEnd - heightCutoffStart);
//      if (scale < 0.0) {
//        scale = 0.0;
//      }
//      l *= scale;
//    }

    l += Eigen::Vector3d(0, 0, zVel);
    state_representation::CartesianTwist velocity("end-effector", l, "robot");
    return velocity;
  }
};

int main(int argc, char** argv) {
  BlendDS DS;

  controllers::impedance::CartesianTwistController ctrl(23, 15, .5, .5);

  std::cout << std::fixed << std::setprecision(3);

  // Set up franka ZMQ
  network::Interface franka(network::InterfaceType::FRANKA_PAPA_16);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};
  command.controlType = frankalwi::proto::JOINT_TORQUE;

  state_representation::CartesianState robot_ee("end-effector", "robot");
  state_representation::Jacobian jacobian("franka", 7, "end-effector", "robot");

  while (franka.receive(state)) {
    frankalwi::utils::toCartesianState(state, robot_ee);
    frankalwi::utils::toJacobian(state.jacobian, jacobian);

    auto twist = DS.blend(state);

    state_representation::JointTorques joint_command = ctrl.compute_command(twist, robot_ee, jacobian);

    frankalwi::utils::fromJointTorque(joint_command, command);
    franka.send(command);
  }
}
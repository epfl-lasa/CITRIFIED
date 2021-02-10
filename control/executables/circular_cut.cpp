#include <vector>
#include <iostream>
#include <cstdio>

#include <state_representation/Space/Cartesian/CartesianPose.hpp>
#include <state_representation/Space/Cartesian/CartesianState.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "controllers/CartesianPoseController.h"
#include "motion_generators/PointAttractorDS.h"
#include "motion_generators/CircularDS.h"
#include "franka_lwi/franka_lwi_utils.h"

class BlendDS {
public:
  BlendDS() {
    orientationDS.setTargetPose(StateRepresentation::CartesianPose("world", center, defaultOrientation));
    orientationDS.linearDS.set_gain(pointGains);

    flatCircleDS = motion_generator::CircleDS(StateRepresentation::CartesianPose("world", center, defaultOrientation));
    flatCircleDS.circularDS.set_radius(radius);
    flatCircleDS.circularDS.set_circular_velocity(radialVelocity);
    flatCircleDS.circularDS.set_planar_gain(circGains[0]);
    flatCircleDS.circularDS.set_normal_gain(circGains[1]);

    inclinedCircleDS = motion_generator::CircleDS(StateRepresentation::CartesianPose("world",
                                                                                     center,
                                                                                     inclination * defaultOrientation));
    inclinedCircleDS.circularDS.set_radius(radius);
    inclinedCircleDS.circularDS.set_circular_velocity(radialVelocity);
    inclinedCircleDS.circularDS.set_planar_gain(circGains[0]);
    inclinedCircleDS.circularDS.set_normal_gain(circGains[1]);
  }

  Eigen::Vector3d center = {0.3, 0.3, 0.555};
  Eigen::Quaterniond defaultOrientation = {0, 0.707, 0.707, 0};//{0.0, 0.919, 0.393, 0.0};
  Eigen::Quaterniond inclination = {0.866, 0.0, 0.5, 0.0};
  double radius = 0.05f;
  double radialVelocity = 1.0f;
  double circGains[2] = {1000.0f, 5.0f};
  std::vector<double> pointGains = {0.0, 0.0, 0.0, 10.0, 10.0, 10.0};

  motion_generator::PointAttractor orientationDS;
  motion_generator::CircleDS flatCircleDS;
  motion_generator::CircleDS inclinedCircleDS;

  std::vector<double> blend(frankalwi::proto::StateMessage<7> state) {
//    updateOrientationTarget(state);
    StateRepresentation::CartesianPose pose(StateRepresentation::CartesianPose::Identity("world"));
    frankalwi::utils::poseFromState(state, pose);
    StateRepresentation::CartesianTwist twist = orientationDS.getTwist(pose);
    // TODO this is just an intermediate solution
    std::vector<double> v1 = {
        twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
        twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
    };
    auto v2 = blendCircles(state);

    std::vector<double> velocity(6, 0);
    for (std::size_t dof = 0; dof < 6; ++dof) {
      if (dof < 3) {
        velocity[dof] = v2[dof];
      } else {
        velocity[dof] = v1[dof];
      }
    }

    return velocity;
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

    //rotate default quaternion about world Z by angle to follow cut in one semicircle,
    // then the opposite way during the restoration phase
    Eigen::Quaterniond
        target = Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())) * defaultOrientation;

    orientationDS.setTargetOrientation(StateRepresentation::CartesianPose("world", center, target));
  }

  std::vector<double> blendCircles(frankalwi::proto::StateMessage<7> state) {
    const double xBlendWidth = 0.1 * radius;
    const double heightCutoffStart = 2 * radius;
    const double heightCutoffEnd = 3 * radius;

    double zVel = 0;

    StateRepresentation::CartesianPose pose(StateRepresentation::CartesianPose::Identity("world"));
    frankalwi::utils::poseFromState(state, pose);
    StateRepresentation::CartesianTwist flatCircleTwist = flatCircleDS.getTwist(pose);
    StateRepresentation::CartesianTwist inclinedCircleTwist = inclinedCircleDS.getTwist(pose);

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

    /*
    if (state.eePose.position.z - center.z() > heightCutoffStart) {
      double scale = heightCutoffEnd - (state.eePose.position.z - center.z()) / (heightCutoffEnd - heightCutoffStart);
      if (scale < 0.0) {
        scale = 0.0;
      }
      l *= scale;
    }
     */

    std::vector<double> v = {l[0], l[1], l[2] + zVel, 0, 0, 0};
//    std::vector<double> v = {l0[0], l0[1], l0[2], 0, 0, 0};
    return v;
  }
};

int main(int argc, char** argv) {
//  BlendDS DS;

  motion_generator::RingDS DS;
  DS.center = {0.35, 0, 0.46};
  DS.inclination = Eigen::Quaterniond(1, 0, 0, 0);
  DS.radius = 0.04;
  DS.width = 0.005;
  DS.speed = 0.045;
  DS.normalGain = 10;
  DS.fieldStrength = 2;
  DS.angularGain = 10;
  DS.maxAngularSpeed = 1.5;

  DS.defaultPose = Eigen::Quaterniond(0.0, -0.393, 0.919, 0.0).normalized();

  controller::CartesianPoseController ctrl(230, 150, 5);
  ctrl.angularController.setDamping(5);

  std::cout << std::fixed << std::setprecision(3);

  // Set up ZMQ
  zmq::context_t context;
  zmq::socket_t publisher, subscriber;
  frankalwi::utils::configureSockets(context, publisher, subscriber);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};
  StateRepresentation::CartesianPose pose(StateRepresentation::CartesianPose::Identity("world"));

  while (subscriber.connected()) {
    if (frankalwi::proto::receive(subscriber, state)) {
//      std::vector<double> desiredVelocity = DS.blend(state);

      network::poseFromState(state, pose);
      StateRepresentation::CartesianTwist twist = DS.getTwist(pose);
      std::vector<double> desiredVelocity = {
          twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
          twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
      };

      command = ctrl.getJointTorque(state, desiredVelocity);
      frankalwi::proto::send(publisher, command);
    }
  }
}
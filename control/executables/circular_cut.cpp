#include <vector>
#include <iostream>
#include <cstdio>

#include <state_representation/Space/Cartesian/CartesianPose.hpp>
#include <state_representation/Space/Cartesian/CartesianState.hpp>

#include <franka_lwi/franka_lwi_communication_protocol.h>

#include "controllers/CartesianPoseController.h"
#include "motion_generators/PointAttractorDS.h"
#include "motion_generators/CircularDS.h"
#include "network/netutils.h"

class BlendDS {
public:
  BlendDS() {
    orientationDS.setTargetPose(StateRepresentation::CartesianPose("world", center, defaultOrientation));
    orientationDS.linearDS.set_gain(pointGains);

    flatCircleDS = motiongenerator::CircleDS(StateRepresentation::CartesianPose("world", center, defaultOrientation));
    flatCircleDS.circularDS.set_radius(radius);
    flatCircleDS.circularDS.set_circular_velocity(radialVelocity);
    flatCircleDS.circularDS.set_planar_gain(circGains[0]);
    flatCircleDS.circularDS.set_normal_gain(circGains[1]);

    inclinedCircleDS = motiongenerator::CircleDS(StateRepresentation::CartesianPose("world",
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

  motiongenerator::PointAttractor orientationDS;
  motiongenerator::CircleDS flatCircleDS;
  motiongenerator::CircleDS inclinedCircleDS;

  std::vector<double> blend(frankalwi::proto::StateMessage<7> state) {
//    updateOrientationTarget(state);
    auto v1 = orientationDS.getTwist(state);
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

    auto v0 = flatCircleDS.getTwist(state);
    Eigen::Vector3d l0(v0[0], v0[1], v0[2]);
    auto v1 = inclinedCircleDS.getTwist(state);
    Eigen::Vector3d l1(v1[0], v1[1], v1[2]);

    Eigen::Vector3d l;
    if (state.eePose.position.x - center.x() > xBlendWidth) {
      l = l0;
      zVel = -0.05;
    } else if (state.eePose.position.x - center.x() < -xBlendWidth) {
      l = l1;
    } else {
      double scale = (state.eePose.position.x - center.x() + xBlendWidth) / (2 * xBlendWidth);
      l = l0 * scale + l1 * (1 - scale);
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
  BlendDS DS;

  controller::CartesianPoseController ctrl(200, 160, 5);
  ctrl.angularController.setDamping(0);

  std::cout << std::fixed << std::setprecision(3);

  // Set up ZMQ
  zmq::context_t context;
  zmq::socket_t publisher, subscriber;
  network::configure(context, publisher, subscriber);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  while (subscriber.connected()) {
    if (frankalwi::proto::receive(subscriber, state)) {
      std::vector<double> desiredVelocity = DS.blend(state);

      command = ctrl.getJointTorque(state, desiredVelocity);
      frankalwi::proto::send(publisher, command);
    }
  }
}

#include <yaml-cpp/yaml.h>

#include "controllers/CartesianPoseController.h"
#include "motion_generators/PointAttractorDS.h"
#include "motion_generators/RingDS.h"
#include "sensors/ForceTorqueSensor.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "franka_lwi/franka_lwi_logger.h"

enum TrialState {
  APPROACH,
  CALIBRATION,
  INSERTION,
  CUT,
  RETRACTION
};

class IncisionTrialSystem {
public:
  IncisionTrialSystem() :
      pointDS(StateRepresentation::CartesianPose::Identity("attractor", "world")),
      ctrl(1, 1, 1) {

    params = YAML::LoadFile("incision_trials_parameters.yaml");
    Eigen::Vector3d center = {
        params["attractor"]["position"]["x"].as<double>(),
        params["attractor"]["position"]["y"].as<double>(),
        params["attractor"]["position"]["z"].as<double>()
    };
    Eigen::Quaterniond defaultOrientation = {
        params["attractor"]["orientation"]["w"].as<double>(),
        params["attractor"]["orientation"]["x"].as<double>(),
        params["attractor"]["orientation"]["y"].as<double>(),
        params["attractor"]["orientation"]["z"].as<double>()
    };
    auto linearGain = params["attractor"]["gains"]["linear"].as<double>();
    auto angularGain = params["attractor"]["gains"]["angular"].as<double>();
    DSgains = {linearGain, linearGain, linearGain, angularGain, angularGain, angularGain};

    pointDS.setTargetPose(StateRepresentation::CartesianPose("world", center, defaultOrientation));
    pointDS.linearDS.set_gain(DSgains);

    ringDS.center = center;
    ringDS.inclination = Eigen::Quaterniond::Identity();
    ringDS.radius = params["circle"]["radius"].as<double>();
    ringDS.width = params["circle"]["width"].as<double>();
    ringDS.speed = params["circle"]["speed"].as<double>();
    ringDS.fieldStrength = params["circle"]["field_strength"].as<double>();
    ringDS.normalGain = params["circle"]["normal_gain"].as<double>();
    ringDS.angularGain = params["circle"]["angular_gain"].as<double>();
    ringDS.maxAngularSpeed = 1.5;

    ctrl = controller::CartesianPoseController(params["default"]["d1"].as<double>(),
                                               params["default"]["d2"].as<double>(),
                                               params["default"]["ak"].as<double>());
    ctrl.angularController.setDamping(params["default"]["ad"].as<double>());
  }

  frankalwi::proto::CommandMessage<7> getCommand(frankalwi::proto::StateMessage<7>& state, TrialState trialState) {
    StateRepresentation::CartesianPose pose(StateRepresentation::CartesianPose::Identity("robot", "world"));
    frankalwi::utils::poseFromState(state, pose);

    StateRepresentation::CartesianTwist twist;
    if (trialState == CUT) {
      twist = ringDS.getTwist(pose);
    } else {
      twist = pointDS.getTwist(pose);
    }

    std::vector<double> desiredVelocity = {
        twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
        twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
    };

    desiredVelocity[2] += zVelocity;

    return ctrl.getJointTorque(state, desiredVelocity);
  }

  void setInsertionPhase() {
    // set the point attractor to ignore Z axis, and instead add constant Z velocity
    DSgains[2] = 0;
    pointDS.linearDS.set_gain(DSgains);
    zVelocity = -params["insertion"]["speed"].as<double>();

    // set the controller to specific insertion phase gains
    ctrl = controller::CartesianPoseController(params["insertion"]["d1"].as<double>(),
                                               params["insertion"]["d2"].as<double>(),
                                               params["insertion"]["ak"].as<double>());
    ctrl.angularController.setDamping(params["insertion"]["ad"].as<double>());
  }

  void setCutPhase(const StateRepresentation::CartesianPose& pose) {
    zVelocity = 0;
    // set circle center from current pose (radial distance along negative Y of local knife orientation)
    std::cout << "Current position: " << pose.get_position().transpose() << std::endl;
    Eigen::Vector3d center(0, -ringDS.radius, 0);
    center = pose.get_orientation().toRotationMatrix() * center + pose.get_position();
    std::cout << "Estimated circle center at " << center.transpose() << std::endl;
  }

  void setRetractionPhase(const StateRepresentation::CartesianPose& pose) {
    // set the point attractor right above the current pose
    auto targetPose = pose;
    auto targetPosition = targetPose.get_position();
    targetPosition.z() += 0.1;
    targetPose.set_position(targetPosition);

    // reset the point attractor DS gains to respect Z axis
    pointDS.setTargetPose(targetPose);
    DSgains[2] = DSgains[1];
    pointDS.linearDS.set_gain(DSgains);
    zVelocity = 0;

    // reset the controller to default damping values
    ctrl = controller::CartesianPoseController(params["default"]["d1"].as<double>(),
                                               params["default"]["d2"].as<double>(),
                                               params["default"]["ak"].as<double>());
    ctrl.angularController.setDamping(params["default"]["ad"].as<double>());
  }

  motion_generator::PointAttractor pointDS;
  motion_generator::RingDS ringDS;
  controller::CartesianPoseController ctrl;
  YAML::Node params;
  std::vector<double> DSgains;
  double zVelocity = 0.0;
};

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // set up control system
  IncisionTrialSystem ITS;

  // set up FT sensor
  sensors::ToolSpec tool;
  tool.mass = 0.08;
  tool.centerOfMass = Eigen::Vector3d(0, 0, 0.025);
  sensors::ForceTorqueSensor ft_sensor("ft_sensor", "192.168.1.1", tool, false, 100);
  StateRepresentation::CartesianWrench rawWrench("ft_sensor_raw", "ft_sensor");
  StateRepresentation::CartesianWrench wrench("ft_sensor", "ft_sensor");
  StateRepresentation::CartesianWrench bias("ft_sensor", "ft_sensor");

  // Set up ZMQ
  zmq::context_t context;
  zmq::socket_t publisher, subscriber;
  frankalwi::utils::configureSockets(context, publisher, subscriber);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};
  StateRepresentation::CartesianPose pose(StateRepresentation::CartesianPose::Identity("robot", "world"));

  StateRepresentation::CartesianPose touchPose = pose;
  bool touchPoseSet = false;

  TrialState trialState = TrialState::APPROACH;
  while (subscriber.connected()) {
    if (frankalwi::proto::receive(subscriber, state)) {
      frankalwi::utils::poseFromState(state, pose);

      if (ft_sensor.readBias(bias)) {
        ft_sensor.readContactWrench(wrench, pose.get_orientation().toRotationMatrix());
      }

      switch (trialState) {
        case APPROACH:
          if ((pose.get_position() - ITS.pointDS.targetPose.get_position()).norm() < 0.05) {
            trialState = CALIBRATION;
            std::cout << "### STARTING CALIBRATION PHASE" << std::endl;
          }
          break;
        case CALIBRATION:
          if (ft_sensor.computeBias(pose.get_orientation().toRotationMatrix(), 500)) {
            ITS.setInsertionPhase();
            trialState = INSERTION;
            std::cout << "### STARTING INSERTION PHASE" << std::endl;
          }
          break;
        case INSERTION: {
          if (wrench.get_force().norm() > ITS.params["insertion"]["max_force"].as<double>()) {
            ITS.setRetractionPhase(pose);
            trialState = RETRACTION;
            std::cout << "### STARTING RETRACTION PHASE" << std::endl;
            break;
          }

          if (!touchPoseSet && wrench.get_force().norm() > ITS.params["insertion"]["touch_force"].as<double>()) {
            touchPose = pose;
            touchPoseSet = true;
          } else {
            double depth = (touchPose.get_position() - pose.get_position()).norm();
            if (depth > ITS.params["insertion"]["depth"].as<double>()) {
              ITS.setCutPhase(pose);
              trialState = CUT;
              std::cout << "### STARTING CUT PHASE" << std::endl;
            }
          }
          break;
        }
        case CUT: {
          double angle = touchPose.get_orientation().angularDistance(pose.get_orientation());
          if (angle > ITS.params["cut"]["arc_angle"].as<double>()) {
            ITS.setRetractionPhase(pose);
            trialState = RETRACTION;
            std::cout << "### STARTING RETRACTION PHASE" << std::endl;
          }
          break;
        }
        case RETRACTION:
          break;
      }

      command = ITS.getCommand(state, trialState);
      frankalwi::proto::send(publisher, command);
    }
  }
}

#include <yaml-cpp/yaml.h>
#include <network/interfaces.h>

#include "controllers/CartesianPoseController.h"
#include "motion_generators/PointAttractorDS.h"
#include "motion_generators/RingDS.h"
#include "sensors/ForceTorqueSensor.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "franka_lwi/franka_lwi_logger.h"
#include "sensors/RigidBodyTracker.h"

#define RB_ID_ROBOT_BASE 1
#define RB_ID_TASK_BASE 2
#define RB_ID_SURFACE_PROBE 3

enum TrialState {
  APPROACH,
  CALIBRATION,
  INSERTION,
  PAUSE,
  CUT,
  RETRACTION
};

class IncisionTrialSystem {
public:
  IncisionTrialSystem() :
      pointDS(StateRepresentation::CartesianPose::Identity("attractor", "world")),
      ctrl(1, 1, 1) {

    params = YAML::LoadFile(std::string(TRIAL_CONFIGURATION_DIR) + "incision_trials_parameters.yaml");
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
    defaultOrientation.normalize();
    auto linearGain = params["attractor"]["gains"]["linear"].as<double>();
    auto angularGain = params["attractor"]["gains"]["angular"].as<double>();
    DSgains = {linearGain, linearGain, linearGain, angularGain, angularGain, angularGain};

    pointDS.setTargetPose(StateRepresentation::CartesianPose("center", center, defaultOrientation, "task"));
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

    // default pose at circle 0 angle (when local Y = 0 and local X > 0)
    //   has the knife X axis pointing along the world Y axis, with Z pointing down
    ringDS.defaultPose = Eigen::Quaterniond(0.0, 0.707, 0.707, 0.0).normalized();

    principleDamping = params["default"]["d1"].as<double>();
    ctrl = controller::CartesianPoseController(principleDamping,
                                               params["default"]["d2"].as<double>(),
                                               params["default"]["ak"].as<double>());
    ctrl.angularController.setDamping(params["default"]["ad"].as<double>());

    cut = !params["insertion_only"].as<bool>();
    trialName = params["trial_prefix"].as<std::string>();
    if (cut) {
      trialName += "cut_" + params["trial"].as<std::string>() + "_"
          + std::to_string(params["insertion"]["speed"].as<double>()) + "_"
          + std::to_string(params["insertion"]["depth"].as<double>()) + "_"
          + std::to_string(params["circle"]["speed"].as<double>()) + "_"
          + std::to_string(params["circle"]["width"].as<double>()) + ".csv";

    } else {
      trialName += "insertion_" + params["trial"].as<std::string>() + "_"
          + std::to_string(params["insertion"]["speed"].as<double>()) + "_"
          + std::to_string(params["insertion"]["depth"].as<double>()) + ".csv";
    }

    std::cout << trialName << std::endl;
  }

  frankalwi::proto::CommandMessage<7> getCommand(frankalwi::proto::StateMessage<7>& state,
                                                 const StateRepresentation::CartesianState& taskInWorld,
                                                 TrialState trialState,
                                                 std::vector<double>& desiredVelocity) {
    StateRepresentation::CartesianPose pose(StateRepresentation::CartesianPose::Identity("robot", "world"));
    frankalwi::utils::poseFromState(state, pose);

    // robot in task
//    pose = StateRepresentation::CartesianPose(taskInWorld.inverse() * pose);
//    std::cout << pose << std::endl;

    StateRepresentation::CartesianTwist twist = StateRepresentation::CartesianTwist::Zero("robot", "task");
    if (trialState == CUT) {
      twist = ringDS.getTwist(pose);
    } else if (trialState != PAUSE) {
      pointDS.linearDS.set_reference_frame(taskInWorld);
      twist = pointDS.getTwist(pose);
    }

    twist.set_linear_velocity(twist.get_linear_velocity() + Eigen::Vector3d(0, 0, zVelocity));
    twist = StateRepresentation::CartesianTwist(taskInWorld * twist);

    desiredVelocity = {
        twist.get_linear_velocity().x(), twist.get_linear_velocity().y(), twist.get_linear_velocity().z(),
        twist.get_angular_velocity().x(), twist.get_angular_velocity().y(), twist.get_angular_velocity().z()
    };
    return ctrl.getJointTorque(state, desiredVelocity);
  }

  void setInsertionPhase() {
    // set the point attractor to ignore Z axis, and instead add constant Z velocity
    DSgains[2] = 0;
    pointDS.linearDS.set_gain(DSgains);
    zVelocity = -params["insertion"]["speed"].as<double>();

    // set the controller to specific insertion phase gains
    principleDamping = params["insertion"]["d1"].as<double>();
    ctrl = controller::CartesianPoseController(principleDamping,
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
    ringDS.center = center;

    // set the controller to specific insertion phase gains
    principleDamping = params["circle"]["d1"].as<double>();
    ctrl = controller::CartesianPoseController(principleDamping,
                                               params["circle"]["d2"].as<double>(),
                                               params["circle"]["ak"].as<double>());
    ctrl.angularController.setDamping(params["circle"]["ad"].as<double>());
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
    principleDamping = params["default"]["d1"].as<double>();
    ctrl = controller::CartesianPoseController(principleDamping,
                                               params["default"]["d2"].as<double>(),
                                               params["default"]["ak"].as<double>());
    ctrl.angularController.setDamping(params["default"]["ad"].as<double>());
  }

  motion_generator::PointAttractor pointDS;
  motion_generator::RingDS ringDS;
  controller::CartesianPoseController ctrl;
  YAML::Node params;
  std::vector<double> DSgains;
  std::string trialName;
  double principleDamping;
  double zVelocity = 0.0;
  bool cut = false;
};

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // set up control system
  IncisionTrialSystem ITS;

  // logger
//  frankalwi::proto::Logger logger(ITS.trialName);

  // set up optitrack
  sensors::RigidBodyTracker optitracker;
  optitracker.start();

  // set up FT sensor
  sensors::ToolSpec tool;
  tool.mass = 0.07;
  tool.centerOfMass = Eigen::Vector3d(0, 0, 0.02);
  sensors::ForceTorqueSensor ft_sensor("ft_sensor", "128.178.145.89", 100, tool, true);
  StateRepresentation::CartesianWrench wrench("ft_sensor_wrench", "ft_sensor");
  StateRepresentation::CartesianWrench bias("ft_sensor_bias", "ft_sensor");

  // set up robot connection
  network::Interface franka(network::InterfaceType::FRANKA_LWI);
  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};
  StateRepresentation::CartesianPose robotInWorld(StateRepresentation::CartesianPose::Identity("robot", "world"));

  StateRepresentation::CartesianState taskInOptitrack(StateRepresentation::CartesianState::Identity("task", "optitrack"));
  StateRepresentation::CartesianState robotInOptitrack(StateRepresentation::CartesianState::Identity("robot", "optitrack"));

  StateRepresentation::CartesianPose touchPose = robotInWorld;
  bool touchPoseSet = false;

  auto start = std::chrono::system_clock::now();
  auto pauseTimer = start;
  TrialState trialState = TrialState::APPROACH;
  while (franka.receive(state)) {
    frankalwi::utils::poseFromState(state, robotInWorld);

    //update optitrack states
    optitracker.getState(robotInOptitrack, RB_ID_ROBOT_BASE);
    optitracker.getState(taskInOptitrack, RB_ID_TASK_BASE);

    if (ft_sensor.readBias(bias)) {
      ft_sensor.readContactWrench(wrench, robotInWorld.get_orientation().toRotationMatrix());
    }

    switch (trialState) {
      case APPROACH:
        if ((robotInWorld.get_position() - ITS.pointDS.targetPose.get_position()).norm() < 0.05) {
//          trialState = CALIBRATION;
//          std::cout << "### STARTING CALIBRATION PHASE" << std::endl;
        }
        break;
      case CALIBRATION:
        if (ft_sensor.computeBias(robotInWorld.get_orientation().toRotationMatrix(), 2000)) {
          ITS.setInsertionPhase();
          trialState = INSERTION;
          std::cout << "### STARTING INSERTION PHASE" << std::endl;
        }
        break;
      case INSERTION: {
        if (wrench.get_force().norm() > ITS.params["insertion"]["max_force"].as<double>()) {
          std::cout << "Max force exceeded" << std::endl;
          ITS.setRetractionPhase(robotInWorld);
          trialState = RETRACTION;
          std::cout << "### STARTING RETRACTION PHASE" << std::endl;
          break;
        }

        if (!touchPoseSet && abs(wrench.get_force().z()) > ITS.params["insertion"]["touch_force"].as<double>()) {
          std::cout << "Surface detected at position " << robotInWorld.get_position().transpose() << std::endl;
          touchPose = robotInWorld;
          touchPoseSet = true;
        } else if (touchPoseSet) {
          double depth = (touchPose.get_position() - robotInWorld.get_position()).norm();
          if (depth > ITS.params["insertion"]["depth"].as<double>()) {
            ITS.zVelocity = 0;
            pauseTimer = std::chrono::system_clock::now();
            trialState = PAUSE;
            std::cout << "### PAUSING - INCISION DEPTH REACHED" << std::endl;
          }
        }
        break;
      }
      case PAUSE: {
        std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - pauseTimer;
        if (elapsed_seconds.count() > 2.0f) {
          if (ITS.cut) {
            ITS.setCutPhase(robotInWorld);
            trialState = CUT;
            std::cout << "### STARTING CUT PHASE" << std::endl;
          } else {
            ITS.setRetractionPhase(robotInWorld);
            trialState = RETRACTION;
            std::cout << "### STARTING RETRACTION PHASE" << std::endl;
          }
          break;
        }
      }
      case CUT: {
        if (wrench.get_force().norm() > ITS.params["insertion"]["max_force"].as<double>()) {
          ITS.setRetractionPhase(robotInWorld);
          trialState = RETRACTION;
          std::cout << "### STARTING RETRACTION PHASE" << std::endl;
          break;
        }

        double angle = touchPose.get_orientation().angularDistance(robotInWorld.get_orientation()) * 180 / M_PI;
        double distance = (robotInWorld.get_position() - touchPose.get_position()).norm();
        if (angle > ITS.params["circle"]["arc_angle"].as<double>() || distance > 0.07) {
          ITS.setRetractionPhase(robotInWorld);
          trialState = RETRACTION;
          std::cout << "### STARTING RETRACTION PHASE" << std::endl;
        }
        break;
      }
      case RETRACTION:
        break;
    }

    // task in world
    auto taskInWorld = robotInWorld * robotInOptitrack.inverse() * taskInOptitrack;
    std::cout << taskInWorld << std::endl;

    std::vector<double> desiredVelocity;
    command = ITS.getCommand(state, taskInWorld, trialState, desiredVelocity);
//    franka.send(command);

    std::array<double, 6> arr{};
    std::copy_n(wrench.array().data(), size(arr), arr.begin());
    state.eeWrench = frankalwi::proto::EETwist(arr);

    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
//    logger.writeCustomLine(elapsed_seconds.count(), state,
//                           desiredVelocity,
//                           std::vector<double>(6, 0),
//                           std::vector<double>(3, 0),
//                           ITS.principleDamping);
  }
}
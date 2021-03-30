#include <yaml-cpp/yaml.h>

#include <dynamical_systems/Linear.hpp>
#include <dynamical_systems/Ring.hpp>

#include "controllers/TwistController.h"
#include "sensors/ForceTorqueSensor.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "sensors/RigidBodyTracker.h"
#include "filters/DigitalButterworth.h"
#include "network/interfaces.h"
#include "logger/JSONLogger.h"
#include "learning/ESNWrapper.h"

#define RB_ID_ROBOT_BASE 1
#define RB_ID_TASK_BASE 2
#define RB_ID_SURFACE_PROBE 3

using namespace state_representation;

enum TrialState {
  APPROACH,
  CALIBRATION,
  TOUCH,
  INSERTION,
  PAUSE,
  CUT,
  RETRACTION
};

static const std::map<TrialState, std::string> trialStateMap {
    {APPROACH, "approach"},
    {CALIBRATION, "calibration"},
    {TOUCH, "touch"},
    {INSERTION, "insertion"},
    {PAUSE, "pause"},
    {CUT, "cut"},
    {RETRACTION, "retraction"}
};

class IncisionTrialSystem {
public:
  IncisionTrialSystem() :
      pointDS(CartesianPose::Identity("center", "task")),
      ringDS(CartesianPose::Identity("center", "task")),
      ctrl(1, 1, 1, 1) {

    params = YAML::LoadFile(std::string(TRIAL_CONFIGURATION_DIR) + "incision_trials_parameters.yaml");

    // Configure linear dynamical system
    CartesianPose center("center", "task");
    center.set_position(params["attractor"]["position"]["x"].as<double>(),
                        params["attractor"]["position"]["y"].as<double>(),
                        params["attractor"]["position"]["z"].as<double>());
    center.set_orientation(Eigen::Quaterniond(params["attractor"]["orientation"]["w"].as<double>(),
                                              params["attractor"]["orientation"]["x"].as<double>(),
                                              params["attractor"]["orientation"]["y"].as<double>(),
                                              params["attractor"]["orientation"]["z"].as<double>()));
    auto linearGain = params["attractor"]["gains"]["linear"].as<double>();
    auto angularGain = params["attractor"]["gains"]["angular"].as<double>();
    DSgains = {linearGain, linearGain, linearGain, angularGain, angularGain, angularGain};

    pointDS.set_attractor(center);
    pointDS.set_gain(DSgains);
    pointDS.set_base_frame(CartesianState::Identity("task", "task"));

    // Configure circular dynamical system
    center.set_orientation(Eigen::Quaterniond::Identity());
    ringDS.set_center(center);
    ringDS.set_radius(params["circle"]["radius"].as<double>());
    ringDS.set_width(params["circle"]["width"].as<double>());
    ringDS.set_speed(params["circle"]["speed"].as<double>());
    ringDS.set_field_strength(params["circle"]["field_strength"].as<double>());
    ringDS.set_normal_gain(params["circle"]["normal_gain"].as<double>());
    ringDS.set_angular_gain(params["circle"]["angular_gain"].as<double>());

    // default pose at circle 0 angle (when local Y = 0 and local X > 0)
    //   has the knife X axis pointing along the world Y axis, with Z pointing down
    ringDS.set_rotation_offset(Eigen::Quaterniond(0.0, 0.707, 0.707, 0.0).normalized());

    // Configure controller
    ctrl = controllers::TwistController(params["default"]["d1"].as<double>(),
                                        params["default"]["d2"].as<double>(),
                                        params["default"]["ak"].as<double>(),
                                        params["default"]["ad"].as<double>());

    cut = !params["insertion_only"].as<bool>();
    trialName = params["trial_prefix"].as<std::string>();
    if (cut) {
      trialName += "cut_" + params["trial"].as<std::string>() + ".json";
    } else {
      trialName += "insertion_" + params["trial"].as<std::string>() + ".json";
    }
    std::cout << trialName << std::endl;
  }

  CartesianTwist getTwistCommand(const CartesianState& eeInTask, const CartesianState& taskInRobot, TrialState trialState) {
    CartesianTwist twistInTask("ee", "task");
    if (trialState == CUT) {
      twistInTask = ringDS.evaluate(eeInTask);
    } else if (trialState != PAUSE) {
      twistInTask = pointDS.evaluate(eeInTask);
    }

    twistInTask.set_linear_velocity(twistInTask.get_linear_velocity() + Eigen::Vector3d(0, 0, zVelocity));
    auto twistInRobot = CartesianTwist(taskInRobot * twistInTask);

    twistInRobot.clamp(params["default"]["max_linear_velocity"].as<double>(),
                       params["default"]["max_angular_velocity"].as<double>());

    return twistInRobot;
  }

  CartesianWrench getWrenchCommand(const CartesianTwist& twistInRobot, const CartesianState& eeInRobot) {
    CartesianWrench wrench = ctrl.compute_command(twistInRobot, eeInRobot);
    wrench.clamp(params["default"]["max_control_force"].as<double>(),
                 params["default"]["max_control_torque"].as<double>());
    return wrench;
  }

  void setDSBaseFrame(const CartesianState& base) {
    pointDS.set_base_frame(base);
    ringDS.set_base_frame(base);
  }

  void setTouchPhase() {
    // set the point attractor to ignore Z axis, and instead add constant Z velocity
    DSgains[2] = 0;
    pointDS.set_gain(DSgains);
    zVelocity = -params["touch"]["speed"].as<double>();

    // set the controller to specific insertion phase gains
    ctrl.set_linear_damping(params["touch"]["d1"].as<double>(), params["touch"]["d2"].as<double>());
    ctrl.angular_stiffness = params["touch"]["ak"].as<double>();
    ctrl.angular_damping = params["touch"]["ad"].as<double>();
  }

  void setInsertionPhase() {
    // set the point attractor to ignore Z axis, and instead add constant Z velocity
    DSgains[2] = 0;
    pointDS.set_gain(DSgains);
    zVelocity = -params["insertion"]["speed"].as<double>();

    // set the controller to specific insertion phase gains
    ctrl.set_linear_damping(params["insertion"]["d1"].as<double>(), params["insertion"]["d2"].as<double>());
    ctrl.angular_stiffness = params["insertion"]["ak"].as<double>();
    ctrl.angular_damping = params["insertion"]["ad"].as<double>();
  }

  void setCutPhase(const CartesianPose& eeInTask) {
    zVelocity = 0;
    // set circle center from current pose (radial distance along negative Y of local knife orientation)
    std::cout << "Current position: " << eeInTask.get_position().transpose() << std::endl;
    Eigen::Vector3d center(0, -ringDS.get_radius(), 0);
    center = eeInTask.get_orientation().toRotationMatrix() * center + eeInTask.get_position();
    center.z() = eeInTask.get_position().z();
    std::cout << "Estimated circle center at " << center.transpose() << std::endl;

    auto centerpose = ringDS.get_center();
    centerpose.set_position(center);
    ringDS.set_center(centerpose);

    // set the controller to specific insertion phase gains
    ctrl.set_linear_damping(params["circle"]["d1"].as<double>(), params["circle"]["d2"].as<double>());
    ctrl.angular_stiffness = params["circle"]["ak"].as<double>();
    ctrl.angular_damping = params["circle"]["ad"].as<double>();
  }

  void setRetractionPhase(const CartesianPose& eeInTask) {
    // set the point attractor right above the current pose
    auto targetPose = eeInTask;
    auto targetPosition = targetPose.get_position();
    targetPosition.z() += 0.1;
    targetPose.set_position(targetPosition);

    // reset the point attractor DS gains to respect Z axis
    pointDS.set_attractor(targetPose);
    DSgains[2] = DSgains[1];
    pointDS.set_gain(DSgains);
    zVelocity = 0;

    // reset the controller to default damping values
    ctrl.set_linear_damping(params["default"]["d1"].as<double>(), params["default"]["d2"].as<double>());
    ctrl.angular_stiffness = params["default"]["ak"].as<double>();
    ctrl.angular_damping = params["default"]["ad"].as<double>();
  }

  // properties
  YAML::Node params;
  std::string trialName;

  dynamical_systems::Linear<CartesianState> pointDS;
  dynamical_systems::Ring ringDS;
  controllers::TwistController ctrl;
  std::vector<double> DSgains;

  double zVelocity = 0.0;
  bool cut = false;
};


int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // set up control system
  IncisionTrialSystem ITS;

  // set up logger
  logger::JSONLogger jsonLogger;
  jsonLogger.addMetaData(ITS.trialName, "details"); //TODO: Copy yaml configuration as string into details
  jsonLogger.write();

  // set up optitrack
  sensors::RigidBodyTracker optitracker;
  optitracker.start();

  // set up FT sensor
  sensors::ToolSpec tool = {
      .centerOfMass = Eigen::Vector3d(0, 0, 0.02),
      .mass = 0.07
  };
  sensors::ForceTorqueSensor ft_sensor("ft_sensor", "128.178.145.89", 100, tool);

  // set up ESN classifier
  const std::vector<std::string> esnInputFields = {"velocity_x", "velocity_z", "force_x", "force_z", "force_derivative_x", "force_derivative_z"};
  const std::string esnConfigFile = std::string(TRIAL_CONFIGURATION_DIR) + "ESN_february591.txt";
  const int esnBufferSize = 50;
  jsonLogger.addSubfield(logger::MessageType::METADATA, "esn", "inputs", esnInputFields);
  jsonLogger.addSubfield(logger::MessageType::METADATA, "esn", "config_file", esnConfigFile);
  jsonLogger.addSubfield(logger::MessageType::METADATA, "esn", "buffer_size", esnBufferSize);
  jsonLogger.addSubfield(logger::MessageType::METADATA, "esn", "sampling_frequency", 500.0);
  Eigen::Vector4d esnInputSample;
  learning::ESNWrapper esn(esnConfigFile, 50);
  esn.setDerivativeCalculationIndices({2, 3});

  // set up filters
  filter::DigitalButterworth twistFilter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 6);
  filter::DigitalButterworth wrenchFilter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 6);

  // set up robot connection
  network::Interface franka(network::InterfaceType::FRANKA_LWI);
  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  // prepare all state objects
  CartesianState eeInRobot("ee", "robot");
  CartesianState taskInOptitrack("task", "optitrack");
  CartesianState robotInOptitrack("robot", "optitrack");
  CartesianWrench ftWrenchInRobot("ft_sensor", "robot");
  CartesianTwist eeLocalTwist("ee", "ee");
  CartesianTwist eeLocalTwistFilt = eeLocalTwist;
  CartesianWrench ftWrenchInRobotFilt = ftWrenchInRobot;

  state_representation::Jacobian jacobian("robot", 7, "ee", "robot");

  CartesianPose touchPose = eeInRobot;
  bool touchPoseSet = false;

  filter::DigitalButterworth filter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 4);
  Eigen::Vector4d filterInput = Eigen::Vector4d::Zero();

  // wait for optitrack data
  std::cout << "Waiting for optitrack robot base and task base state..." << std::endl;
  while (!optitracker.getState(robotInOptitrack, RB_ID_ROBOT_BASE)
      || !optitracker.getState(taskInOptitrack, RB_ID_TASK_BASE)) {}

  std::cout << "Optitrack ready" << std::endl;

  // start main control loop
  int iterations = 0;
  auto frequencyTimer = std::chrono::system_clock::now();
  auto pauseTimer = std::chrono::system_clock::now();
  TrialState trialState = TrialState::APPROACH;
  while (franka.receive(state)) {

    //  update states
    frankalwi::utils::toCartesianState(state, eeInRobot);
    frankalwi::utils::toJacobian(state.jacobian, jacobian);
    eeLocalTwist.set_twist((-1.0 * CartesianTwist(eeInRobot).inverse()).get_twist());

    // update optitrack states
    optitracker.getState(robotInOptitrack, RB_ID_ROBOT_BASE);
    optitracker.getState(taskInOptitrack, RB_ID_TASK_BASE);
    auto taskInRobot = robotInOptitrack.inverse() * taskInOptitrack;
    auto eeInTask = taskInRobot.inverse() * eeInRobot;
    ITS.setDSBaseFrame(taskInRobot);

    // update ft wrench
    if (ft_sensor.biasOK()) {
      ft_sensor.readContactWrench(ftWrenchInRobot, eeInRobot.get_orientation().toRotationMatrix());
    }

    // filter data
    eeLocalTwistFilt.set_twist(twistFilter.computeFilterOutput(eeLocalTwist.get_twist()));
    ftWrenchInRobotFilt.set_wrench(wrenchFilter.computeFilterOutput(ftWrenchInRobot.get_wrench()));

    if (ftWrenchInRobot.get_force().norm() > ITS.params["default"]["max_force"].as<double>()) {
      std::cout << "Max force exceeded" << std::endl;
      ITS.setRetractionPhase(eeInTask);
      trialState = RETRACTION;
      std::cout << "### STARTING RETRACTION PHASE" << std::endl;
      break;
    }

    switch (trialState) {
      case APPROACH:
        if ((eeInTask.get_position() - ITS.pointDS.get_attractor().get_position()).norm() < 0.05) {
          trialState = CALIBRATION;
          std::cout << "### STARTING CALIBRATION PHASE" << std::endl;
        }
        break;
      case CALIBRATION:
        if (ft_sensor.computeBias(eeInRobot.get_orientation().toRotationMatrix(), 2000)) {
          ITS.setTouchPhase();
          trialState = TOUCH;
          std::cout << "### STARTING TOUCH PHASE" << std::endl;
        }
        break;
      case TOUCH:
        if (!touchPoseSet && abs(ftWrenchInRobotFilt.get_force().z()) > ITS.params["touch"]["touch_force"].as<double>()) {
          std::cout << "Surface detected at position " << eeInRobot.get_position().transpose() << std::endl;
          touchPose = eeInRobot;
          touchPoseSet = true;

          std::cout << "Starting ESN thread" << std::endl;
          esn.start();

          ITS.setInsertionPhase();
          trialState = INSERTION;
          std::cout << "### STARTING INSERTION PHASE" << std::endl;
        }
        break;
      case INSERTION: {
        double depth = (touchPose.get_position() - eeInRobot.get_position()).norm();
        jsonLogger.addField(logger::MODEL, "depth", depth);
        if (depth > ITS.params["insertion"]["depth"].as<double>()) {
          ITS.zVelocity = 0;

          std::cout << "Stopping ESN thread" << std::endl;
          esn.stop();

          pauseTimer = std::chrono::system_clock::now();
          trialState = PAUSE;
          std::cout << "### PAUSING - INCISION DEPTH REACHED" << std::endl;
        }

        // combine sample for esn input
        esnInputSample << depth,
            eeLocalTwistFilt.get_linear_velocity().x(),
            eeLocalTwistFilt.get_linear_velocity().z(),
            ftWrenchInRobotFilt.get_force().x(),
            ftWrenchInRobotFilt.get_force().z();

        esn.addSample(jsonLogger.getTime(), esnInputSample);
        Eigen::MatrixXd timeBuffer, dataBuffer;
        if (auto prediction = esn.getLastPrediction(timeBuffer, dataBuffer)) {
          std::vector<double> probabilities;
          Eigen::Map<Eigen::VectorXd>(probabilities.data(), 1, prediction->predictions.size()) = prediction->predictions;
          jsonLogger.addField(logger::MessageType::ESN, "probabilities", probabilities);
          jsonLogger.addField(logger::MessageType::ESN, "class_index", prediction->classIndex);
          jsonLogger.addField(logger::MessageType::ESN, "class_name", prediction->className);

          // TODO: add utility to jsonlogger or esnwrapper to make casting Eigen to json objects easier
          std::vector<double> times;
          Eigen::Map<Eigen::MatrixXd>(times.data(), timeBuffer.rows(), 1) = timeBuffer;
          jsonLogger.addSubfield(logger::MessageType::ESN, "input", "time", times);

          std::vector<double> data;
          Eigen::Map<Eigen::MatrixXd>(data.data(), dataBuffer.rows(), 1) = dataBuffer.row(0);
          jsonLogger.addSubfield(logger::MessageType::ESN, "input", "depth", data);
          Eigen::Map<Eigen::MatrixXd>(data.data(), dataBuffer.rows(), 1) = dataBuffer.row(1);
          jsonLogger.addSubfield(logger::MessageType::ESN, "input", "velocity_x", data);
          Eigen::Map<Eigen::MatrixXd>(data.data(), dataBuffer.rows(), 1) = dataBuffer.row(2);
          jsonLogger.addSubfield(logger::MessageType::ESN, "input", "velocity_z", data);
          Eigen::Map<Eigen::MatrixXd>(data.data(), dataBuffer.rows(), 1) = dataBuffer.row(3);
          jsonLogger.addSubfield(logger::MessageType::ESN, "input", "force_x", data);
          Eigen::Map<Eigen::MatrixXd>(data.data(), dataBuffer.rows(), 1) = dataBuffer.row(4);
          jsonLogger.addSubfield(logger::MessageType::ESN, "input", "force_z", data);
          Eigen::Map<Eigen::MatrixXd>(data.data(), dataBuffer.rows(), 1) = dataBuffer.row(5);
          jsonLogger.addSubfield(logger::MessageType::ESN, "input", "force_derivative_x", data);
          Eigen::Map<Eigen::MatrixXd>(data.data(), dataBuffer.rows(), 1) = dataBuffer.row(6);
          jsonLogger.addSubfield(logger::MessageType::ESN, "input", "force_derivative_z", data);
        }
        break;
      }
      case PAUSE: {
        std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - pauseTimer;
        if (elapsed_seconds.count() > 2.0f) {
          if (ITS.cut) {
            ITS.setCutPhase(eeInTask);
            trialState = CUT;
            std::cout << "### STARTING CUT PHASE" << std::endl;
          } else {
            ITS.setRetractionPhase(eeInTask);
            trialState = RETRACTION;
            std::cout << "### STARTING RETRACTION PHASE" << std::endl;
          }
        }
        break;
      }
      case CUT: {
        double angle = touchPose.get_orientation().angularDistance(eeInRobot.get_orientation()) * 180 / M_PI;
        double distance = (eeInRobot.get_position() - touchPose.get_position()).norm();
        if (angle > ITS.params["circle"]["arc_angle"].as<double>() || distance > 0.07) {
          ITS.setRetractionPhase(eeInTask);
          trialState = RETRACTION;
          std::cout << "### STARTING RETRACTION PHASE" << std::endl;
        }
        break;
      }
      case RETRACTION:
        break;
    }

    CartesianTwist commandTwistInRobot = ITS.getTwistCommand(eeInTask, taskInRobot, trialState);
    CartesianWrench commandWrenchInRobot = ITS.getWrenchCommand(commandTwistInRobot, eeInRobot);

    frankalwi::utils::fromJointTorque(jacobian.transpose() * commandWrenchInRobot, command);
    franka.send(command);

    jsonLogger.addTime();
    jsonLogger.addBody(logger::RAW, eeInRobot);
    jsonLogger.addBody(logger::RAW, eeLocalTwist);
    jsonLogger.addBody(logger::RAW, taskInRobot);
    jsonLogger.addBody(logger::RAW, ftWrenchInRobot);

    jsonLogger.addBody(logger::FILTERED, eeInRobot);
    jsonLogger.addBody(logger::FILTERED, eeLocalTwistFilt);
    jsonLogger.addBody(logger::FILTERED, ftWrenchInRobotFilt);

    jsonLogger.addCommand(commandTwistInRobot, commandWrenchInRobot);
    jsonLogger.addField(logger::CONTROL, "phase", trialStateMap.at(trialState));
    jsonLogger.addField(logger::CONTROL, "gains", std::vector<double>({
      ITS.ctrl.get_linear_damping(0),
      ITS.ctrl.get_linear_damping(1),
      ITS.ctrl.angular_stiffness,
      ITS.ctrl.angular_damping
    }));

    jsonLogger.write();

    // frequency
    ++iterations;
    if (iterations > 1000) {
      std::chrono::duration<double> delta = std::chrono::system_clock::now() - frequencyTimer;
      std::cout << double(1000) / delta.count() << " Hz" << std::endl;
      frequencyTimer = std::chrono::system_clock::now();
      iterations = 0;
    }
  }
}
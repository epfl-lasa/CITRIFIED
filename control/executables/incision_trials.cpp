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
#include "learning/ESN.h"

#define RB_ID_ROBOT_BASE 1
#define RB_ID_TASK_BASE 2
#define RB_ID_SURFACE_PROBE 3

using namespace state_representation;

enum TrialState {
  APPROACH,
  CALIBRATION,
  INSERTION,
  PAUSE,
  CUT,
  RETRACTION
};

static const std::map<TrialState, std::string> trialStateMap {
    {APPROACH, "approach"},
    {CALIBRATION, "calibration"},
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

/*
class ESNWrapper {
public:
  explicit ESNWrapper(const std::string& file) :
      esn_(file),
      window_(Eigen::Matrix<double, 50, 6>::Zero()),
      windowCopy_(window_) {
    labels.insert_or_assign(0, "None");
    labels.insert_or_assign(1, "Air");
    labels.insert_or_assign(2, "Apple");
    labels.insert_or_assign(3, "Banana");
    labels.insert_or_assign(4, "Orange");
  }

  void start() {
    keepAlive_ = true;
    esnThread_ = std::thread([this] { runClassifier(); });
  }

  void stop() {
    keepAlive_ = false;
    esnThread_.join();
  }
  int classifyOnce() {
    return esn_.test_esn(window_, window_.cols());
  }

  int getLabel() {
    if (ready()) {
      return label;
    }
    return 0;
  }

  void addSample(const Eigen::VectorXd& sample) {
    // shift the sample observations up one row, and append the most recent sample to the end
    esnMutex_.lock();
    window_.block<49, 6>(0, 0) = Eigen::Matrix<double, 49, 6>(window_.block<49, 6>(1, 0));
    window_.row(window_.rows() - 1) = sample;
    esnMutex_.unlock();

    if (nSamples_ < window_.rows()) {
      ++nSamples_;
    }
  }

  bool ready() {
    return samplesReady() && labelReady_;
  }

  int checkTick() {
    if (eval_) {
      eval_ = false;
      return 1;
    }
    return 0;
  }

  int label = 0;
private:
  std::map<int, std::string> labels;
  bool keepAlive_ = false;
  bool labelReady_ = false;
  bool eval_ = false;
  std::thread esnThread_;
  std::mutex esnMutex_;
  learning::ESN esn_;
  Eigen::Matrix<double, 50, 6> window_, windowCopy_;
  int nSamples_ = 0;

  void runClassifier() {
    while (keepAlive_) {
      if (samplesReady()) {
        esnMutex_.lock();
        windowCopy_ = window_;
        esnMutex_.unlock();
        label = esn_.test_esn(windowCopy_, windowCopy_.cols());
        labelReady_ = true;
        eval_ = true;
        if (labels.count(label)) {
//          std::cout << labels.at(label) << std::endl;
        }
      }
    }
  }

  bool samplesReady() {
    return nSamples_ >= window_.rows();
  }
};
*/

Eigen::VectorXd secondOrderDerivative(const Eigen::MatrixXd& data, const Eigen::VectorXd& time) {
  Eigen::Vector2d derivative = (data.row(0) - data.row(2)) / (time(0) - time(2));
  return derivative;
}

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
  sensors::ForceTorqueSensor ft_sensor("ft_sensor", "128.178.145.248", 100, tool);

/*
  // set up ESN classifier
  ESNWrapper esn(std::string(TRIAL_CONFIGURATION_DIR) + "ESN_february591.txt");
  esn.start();
*/

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

  state_representation::Jacobian jacobian("robot", 7);

  CartesianPose touchPose = eeInRobot;
  bool touchPoseSet = false;

  Eigen::Matrix<double, 3, 7> esnSignalsWithTime = Eigen::Matrix<double, 3, 7>::Zero();
  filter::DigitalButterworth filter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 4);
  Eigen::Vector4d filterInput = Eigen::Vector4d::Zero();

  // wait for optitrack data
  std::cout << "Waiting for optitrack robot base and task base state..." << std::endl;
  while (!optitracker.getState(robotInOptitrack, RB_ID_ROBOT_BASE)
      || !optitracker.getState(taskInOptitrack, RB_ID_TASK_BASE)) {}

  std::cout << "Optitrack ready" << std::endl;

  // start main control loop
  int iterations = 0;
  auto start = std::chrono::system_clock::now();
  auto frequencyTimer = start;
  auto esnTimer = start;
  auto pauseTimer = start;
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

/*
    // filter data for ESN
    Eigen::Vector3d velocityExpressedInEEFrame = eeInRobot.get_orientation().toRotationMatrix()
        * Eigen::Vector3d(frankalwi::proto::vec3DToArray(state.eeTwist.linear).data());
    filterInput =
        Eigen::Vector4d(velocityExpressedInEEFrame.x(), velocityExpressedInEEFrame.z(), ftWrenchInRobot.get_force().x(),
                        ftWrenchInRobot.get_force().z());
    esnSignalsWithTime.topRows(2) = esnSignalsWithTime.bottomRows(2);
    esnSignalsWithTime.row(2).head(4) = filter.computeFilterOutput(filterInput);
    std::chrono::duration<double> deltaT = std::chrono::system_clock::now() - esnTimer;
    esnSignalsWithTime.row(2)(6) = deltaT.count();
    esnSignalsWithTime.row(2).segment(4, 2) =
        secondOrderDerivative(esnSignalsWithTime.block<3, 2>(0, 2), esnSignalsWithTime.col(6));
    esnTimer = std::chrono::system_clock::now();

    // update ESN data
    esn.addSample(esnSignalsWithTime.block<1, 6>(2, 0));
*/

    switch (trialState) {
      case APPROACH:
        if ((eeInTask.get_position() - ITS.pointDS.get_attractor().get_position()).norm() < 0.05) {
          trialState = CALIBRATION;
          std::cout << "### STARTING CALIBRATION PHASE" << std::endl;
        }
        break;
      case CALIBRATION:
        if (ft_sensor.computeBias(eeInRobot.get_orientation().toRotationMatrix(), 2000)) {
          ITS.setInsertionPhase();
          trialState = INSERTION;
          std::cout << "### STARTING INSERTION PHASE" << std::endl;
        }
        break;
      case INSERTION: {
        if (ftWrenchInRobot.get_force().norm() > ITS.params["insertion"]["max_force"].as<double>()) {
          std::cout << "Max force exceeded" << std::endl;
          ITS.setRetractionPhase(eeInTask);
          trialState = RETRACTION;
          std::cout << "### STARTING RETRACTION PHASE" << std::endl;
          break;
        }

        if (!touchPoseSet
            && abs(ftWrenchInRobot.get_force().z()) > ITS.params["insertion"]["touch_force"].as<double>()) {
          std::cout << "Surface detected at position " << eeInRobot.get_position().transpose() << std::endl;
          touchPose = eeInRobot;
          touchPoseSet = true;
        } else if (touchPoseSet) {
          double depth = (touchPose.get_position() - eeInRobot.get_position()).norm();
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
            ITS.setCutPhase(eeInTask);
            trialState = CUT;
            std::cout << "### STARTING CUT PHASE" << std::endl;
          } else {
            ITS.setRetractionPhase(eeInTask);
            trialState = RETRACTION;
            std::cout << "### STARTING RETRACTION PHASE" << std::endl;
          }
          break;
        }
      }
      case CUT: {
        if (ftWrenchInRobot.get_force().norm() > ITS.params["insertion"]["max_force"].as<double>()) {
          ITS.setRetractionPhase(eeInTask);
          trialState = RETRACTION;
          std::cout << "### STARTING RETRACTION PHASE" << std::endl;
          break;
        }

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

    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;

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

    if (touchPoseSet) {
      jsonLogger.addField(logger::MODEL, "depth", (eeInRobot - touchPose).get_position().z());
    }

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
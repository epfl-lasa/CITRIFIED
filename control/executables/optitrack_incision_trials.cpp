#include "controllers/IncisionTrialSystem.h"
#include "controllers/CutProber.h"

#include "sensors/ForceTorqueSensor.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "sensors/RigidBodyTracker.h"
#include "filters/DigitalButterworth.h"
#include "network/interfaces.h"
#include "logger/JSONLogger.h"
#include "learning/ESNWrapper.h"
#include "learning/GPR.h"

#define RB_ID_ROBOT_BASE 1
#define RB_ID_TASK_BASE 2

using namespace state_representation;

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // set up control system
  auto configFile = std::string(TRIAL_CONFIGURATION_DIR) + "incision_trials_parameters.yaml";
  IncisionTrialSystem ITS(configFile);
  CutProber CP(configFile);
  CP.setStart(ITS.pointDS.get_attractor());

  std::cout << ITS.trialName << std::endl;

  // set up logger
  logger::JSONLogger jsonLogger(ITS.trialName);
  jsonLogger.addMetaData(ITS.trialName, ITS.yamlContent);
  jsonLogger.addSubfield(logger::METADATA, "insertion", "depth", ITS.params["insertion"]["depth"].as<double>());
  jsonLogger.addSubfield(logger::METADATA, "insertion", "pitch", ITS.params["attractor"]["pitch_angle"].as<double>());
  if (ITS.cut) {
    jsonLogger.addSubfield(logger::METADATA, "cut", "depth", ITS.params["cut"]["depth"].as<double>());
    jsonLogger.addSubfield(logger::METADATA, "cut", "radius", ITS.params["cut"]["radius"].as<double>());
    jsonLogger.addSubfield(logger::METADATA, "cut", "speed", ITS.params["cut"]["speed"].as<double>());
    jsonLogger.addSubfield(logger::METADATA, "cut", "normal_gain", ITS.params["cut"]["normal_gain"].as<double>());
  }

  // set up optitrack
  sensors::RigidBodyTracker optitracker;
  optitracker.start();

  // set up FT sensor
  sensors::ToolSpec tool = {
      .centerOfMass = Eigen::Vector3d(0, 0, 0.02), .mass = 0.07
  };
  sensors::ForceTorqueSensor ft_sensor("ft_sensor", "128.178.145.248", 100, tool);

  CartesianState taskInOptitrack("task", "optitrack");
  CartesianState robotInOptitrack("robot", "optitrack");
  // wait for optitrack data
  std::cout << "Waiting for optitrack robot base and task base state..." << std::endl;
  while (!optitracker.getState(robotInOptitrack, RB_ID_ROBOT_BASE)
      || !optitracker.getState(taskInOptitrack, RB_ID_TASK_BASE)) {}
  std::cout << "Optitrack ready" << std::endl;

  // set up GPR predictor
  std::cout << "Waiting for GPR server..." << std::endl;
  learning::GPR<2> gpr;
  bool gprStarted = false;
  gpr.testConnection();
  std::cout << "GPR server ready" << std::endl;

  // set up ESN classifier
  std::cout << "Initializing ESN..." << std::endl;
  const std::vector<std::string> esnInputFields =
      {"depth", "velocity_x", "velocity_z", "force_x", "force_z", "force_derivative_x", "force_derivative_z"};
  const std::string esnConfigFile = std::string(TRIAL_CONFIGURATION_DIR) + ITS.esnFilename;
  int esnSkip = 2;
  jsonLogger.addSubfield(logger::MessageType::METADATA, "esn", "inputs", esnInputFields);
  jsonLogger.addSubfield(logger::MessageType::METADATA, "esn", "config_file", esnConfigFile);
  jsonLogger.addSubfield(logger::MessageType::METADATA, "esn", "buffer_size", ITS.esnBufferSize);
  jsonLogger.addSubfield(logger::MessageType::METADATA,
                         "esn",
                         "min_time_between_predictions",
                         ITS.esnMinTimeBetweenPredictions);
  jsonLogger.addSubfield(logger::MessageType::METADATA, "esn", "sampling_frequency", 500.0);
  Eigen::VectorXd esnInputSample(5);
  learning::ESNWrapper esn(esnConfigFile, ITS.esnBufferSize, ITS.esnMinTimeBetweenPredictions);
  esn.setDerivativeCalculationIndices({3, 4});
  std::vector<learning::esnPrediction> esnPredictionCollection;
  learning::esnPrediction finalESNPrediction;
  std::cout << "ESN ready" << std::endl;

  // set up filters
  filter::DigitalButterworth twistFilter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 6);
  filter::DigitalButterworth wrenchFilter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 6);
  filter::DigitalButterworth stateFilter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 13);

  // set up robot connection
  network::Interface franka(network::InterfaceType::FRANKA_PAPA_16);
  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};
  command.controlType = frankalwi::proto::JOINT_TORQUE;

  // prepare all state objects
  CartesianState eeInRobot("ee", "robot");
  CartesianState eeInRobotFilt = eeInRobot;
  CartesianWrench ftWrenchInRobot("ft_sensor", "robot");
  CartesianTwist eeLocalTwist("ee", "ee");
  CartesianTwist eeLocalTwistFilt = eeLocalTwist;
  CartesianWrench ftWrenchInRobotFilt = ftWrenchInRobot;

  state_representation::Jacobian jacobian("franka", 7, "ee", "robot");

  CartesianPose touchPose = eeInRobot;
  bool touchPoseSet = false;

  filter::DigitalButterworth filter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 4);
  auto filterInput = Eigen::VectorXd::Zero(4);

  jsonLogger.addBody(logger::MessageType::STATIC, robotInOptitrack);
  jsonLogger.addBody(logger::MessageType::STATIC, taskInOptitrack);
  jsonLogger.write();

  // adaptive cut parameters
  int cutStable = -ITS.params["cut"]["evaluation_buffer"].as<int>();
  auto deviationOffset = ITS.params["cut"]["deviation_offset"].as<double>();
  auto deviationRate = ITS.params["cut"]["offset_incremental_rate"].as<double>();
  std::array<double, 2> maxDeviation = {0, 0};

  std::cout << "Ready to begin trial!" << std::endl;
  // start main control loop
  bool finished = false;
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

    // filter data: robot state
    Eigen::Matrix<double, 13, 1> stateVector;
    stateVector << eeInRobot.get_pose(), eeInRobot.get_twist();
    auto stateFilt = stateFilter.computeFilterOutput(stateVector);
    eeInRobotFilt.set_pose(stateFilt.head(7));
    eeInRobotFilt.set_twist(stateFilt.tail(6));

    // filter data: localised twist and force
    eeLocalTwistFilt.set_twist(twistFilter.computeFilterOutput(eeLocalTwist.get_twist()));
    ftWrenchInRobotFilt.set_wrench(wrenchFilter.computeFilterOutput(ftWrenchInRobot.get_wrench()));

    if (ftWrenchInRobot.get_force().norm() > ITS.params["default"]["max_force"].as<double>()) {
      std::cout << "Max force exceeded" << std::endl;
      CP.full = true;
      ITS.setRetractionPhase(eeInTask);
      finished = true;
      trialState = RETRACTION;
      std::cout << "### ENTERING FINAL RETRACTION PHASE" << std::endl;
      break;
    }

    switch (trialState) {
      case APPROACH:
        if ((eeInTask.get_position() - ITS.pointDS.get_attractor().get_position()).norm() < 0.02) {
          pauseTimer = std::chrono::system_clock::now();
          trialState = CALIBRATION;
          std::cout << "### STARTING CALIBRATION PHASE" << std::endl;
        }
        break;
      case CALIBRATION: {
        std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - pauseTimer;
        if (elapsed_seconds.count() > 2.0f) {
          if (ft_sensor.computeBias(eeInRobot.get_orientation().toRotationMatrix(), 2000)) {
            ITS.setTouchPhase();
            trialState = TOUCH;
            std::cout << "### STARTING TOUCH PHASE" << std::endl;
          }
        }
        break;
      }
      case TOUCH:
        if (abs(ftWrenchInRobotFilt.get_force().z()) > ITS.params["touch"]["touch_force"].as<double>()) {
          std::cout << "Surface detected at position " << eeInRobot.get_position().transpose() << std::endl;

          jsonLogger.addField(logger::MODEL, "touch_position", std::vector<double>({
                                                                                       eeInTask.get_position().x(),
                                                                                       eeInTask.get_position().y(),
                                                                                       eeInTask.get_position().z()
                                                                                   }));

          // if we need more touch points, record this one and go back to approach the next one
          if (!CP.full && ITS.cut) {
            printf("Adding touch position to cut prober (total samples: %i)\n", CP.count());
            CP.addPoint(eeInTask);

            if (CP.count() < ITS.params["probe"]["samples"].as<int>()
                && CP.angle() < ITS.params["cut"]["arc_angle"].as<double>() * 180 / M_PI
                && CP.dist() < ITS.params["cut"]["cut_distance"].as<double>()) {
              ITS.setRetractionPhase(eeInTask, 0.0);
            } else {
              CP.full = true;
              ITS.setRetractionPhase(eeInTask);
            }

            trialState = RETRACTION;
            std::cout << "### STARTING RETRACTION PHASE" << std::endl;

          } else {
            std::cout << "Starting ESN thread" << std::endl;
            esn.start();

            touchPose = eeInRobot;
            touchPoseSet = true;

            ITS.setInsertionPhase();
            trialState = INSERTION;
            std::cout << "### STARTING INSERTION PHASE" << std::endl;
          }
        }
        break;
      case INSERTION: {
        double depth = 0;
        if (ITS.cut) {
          depth = CP.estimateHeightInTask(eeInTask) - eeInTask.get_position().z();
        } else {
          depth = (touchPose.get_position() - eeInRobot.get_position()).norm();
        }
        jsonLogger.addField(logger::MODEL, "depth", depth);
        if (depth > ITS.params["insertion"]["depth"].as<double>()) {
          ITS.zVelocity = 0;

          std::cout << "Stopping ESN thread" << std::endl;
          esn.stop();

          // hold the current position
          ITS.setRetractionPhase(eeInTask,
                                 ITS.params["insertion"]["depth"].as<double>()
                                     - ITS.params["cut"]["depth"].as<double>());
          pauseTimer = std::chrono::system_clock::now();
          trialState = CLASSIFICATION;
          std::cout << "### CLASSIFYING - INCISION DEPTH REACHED" << std::endl;
        }

        esnSkip--;
        if (esnSkip <= 0) {
          // combine sample for esn input
          esnInputSample
              << depth, eeLocalTwistFilt.get_linear_velocity().x(), eeLocalTwistFilt.get_linear_velocity().z(), ftWrenchInRobotFilt.get_force().x(), ftWrenchInRobotFilt.get_force().z();

          esn.addSample(jsonLogger.getTime(), esnInputSample);
          esnSkip = 2;
        }
        Eigen::MatrixXd timeBuffer, dataBuffer;
        auto esnPrediction = esn.getLastPrediction(timeBuffer, dataBuffer);
        if (esnPrediction && esnPredictionCollection.size() < 3) {
          esnPredictionCollection.emplace_back(*esnPrediction);
          std::vector<double> probabilities(esnPrediction->predictions.data(),
                                            esnPrediction->predictions.data() + esnPrediction->predictions.size());
          jsonLogger.addField(logger::MessageType::ESN, "probabilities", probabilities);
          jsonLogger.addField(logger::MessageType::ESN, "class_index", esnPrediction->classIndex);
          jsonLogger.addField(logger::MessageType::ESN, "class_name", esnPrediction->className);

          jsonLogger.addSubfield(logger::MessageType::ESN,
                                 "input",
                                 "time",
                                 std::vector<double>(timeBuffer.data(), timeBuffer.data() + timeBuffer.size()));
          jsonLogger.addSubfield(logger::MessageType::ESN,
                                 "input",
                                 "depth",
                                 std::vector<double>(dataBuffer.col(0).data(),
                                                     dataBuffer.col(0).data() + ITS.esnBufferSize));
          jsonLogger.addSubfield(logger::MessageType::ESN,
                                 "input",
                                 "velocity_x",
                                 std::vector<double>(dataBuffer.col(1).data(),
                                                     dataBuffer.col(1).data() + ITS.esnBufferSize));
          jsonLogger.addSubfield(logger::MessageType::ESN,
                                 "input",
                                 "velocity_z",
                                 std::vector<double>(dataBuffer.col(2).data(),
                                                     dataBuffer.col(2).data() + ITS.esnBufferSize));
          jsonLogger.addSubfield(logger::MessageType::ESN,
                                 "input",
                                 "force_x",
                                 std::vector<double>(dataBuffer.col(3).data(),
                                                     dataBuffer.col(3).data() + ITS.esnBufferSize));
          jsonLogger.addSubfield(logger::MessageType::ESN,
                                 "input",
                                 "force_z",
                                 std::vector<double>(dataBuffer.col(4).data(),
                                                     dataBuffer.col(4).data() + ITS.esnBufferSize));
          jsonLogger.addSubfield(logger::MessageType::ESN,
                                 "input",
                                 "force_derivative_x",
                                 std::vector<double>(dataBuffer.col(5).data(),
                                                     dataBuffer.col(5).data() + ITS.esnBufferSize));
          jsonLogger.addSubfield(logger::MessageType::ESN,
                                 "input",
                                 "force_derivative_z",
                                 std::vector<double>(dataBuffer.col(6).data(),
                                                     dataBuffer.col(6).data() + ITS.esnBufferSize));
        }
        break;
      }
      case CLASSIFICATION: {
        if (esnPredictionCollection.size() < 3) {
          std::cout << "less than 3 predictions available! (" << esnPredictionCollection.size() << ")" << std::endl;
        }
        if (!esnPredictionCollection.empty()) {
          finalESNPrediction = esn.getFinalClass(esnPredictionCollection);
          std::vector<double> probabilities(finalESNPrediction.predictions.data(),
                                            finalESNPrediction.predictions.data()
                                                + finalESNPrediction.predictions.size());
          jsonLogger.addField(logger::MessageType::ESN, "probabilities", probabilities);
          jsonLogger.addField(logger::MessageType::ESN, "class_index", finalESNPrediction.classIndex);
          jsonLogger.addField(logger::MessageType::ESN, "class_name", finalESNPrediction.className);
        }
        esn.stop();
        trialState = PAUSE;
        std::cout << "### PAUSING - TISSUE CLASSIFIED : " << finalESNPrediction.className << std::endl;
        break;
      }
      case PAUSE: {
        bool valid = false;
        if (!valid) {
          for (auto& permitted : ITS.permittedClasses) {
            if (permitted == finalESNPrediction.className || permitted == "all") {
              valid = true;
              break;
            }
          }
          if (!valid) {
            finished = true;
            ITS.setRetractionPhase(eeInTask);
            trialState = RETRACTION;
            std::cout << "### INVALID CLASS TYPE" << std::endl;
            std::cout << "### STARTING RETRACTION PHASE" << std::endl;
          }
        }

        if (ITS.cut && !gprStarted) {
          std::cout << "Starting GPR server" << std::endl;
          gprStarted = gpr.start(finalESNPrediction.classIndex);
          // pre-load the GPR model by sending a dummy packet
          gpr.updateState(std::array<double, 2>{0, 0});
        }

        std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - pauseTimer;
        if (elapsed_seconds.count() > 2.0f) {
          if (ITS.cut && gprStarted) {
            ITS.setCutPhase(eeInTask);
            trialState = CUT;
            std::cout << "### STARTING CUT PHASE" << std::endl;
          } else {
            finished = true;
            ITS.setRetractionPhase(eeInTask);
            trialState = RETRACTION;
            std::cout << "### STARTING RETRACTION PHASE" << std::endl;
          }
        }
        break;
      }
      case CUT: {
        auto center = ITS.ringDS.get_center();
        auto position = center.get_position();
        position.z() = CP.estimateHeightInTask(eeInTask);
        double depth = position.z() - eeInTask.get_position().z();
        jsonLogger.addField(logger::MODEL, "depth", depth);
        position.z() -= ITS.params["cut"]["depth"].as<double>();
        center.set_position(position);
        ITS.ringDS.set_center(center);

        std::array<double, 2> gprRequest = {depth, eeInRobotFilt.get_linear_velocity().x()};
        gpr.updateState(gprRequest);
        if (auto gprPrediction = gpr.getLastPrediction()) {
          double deviation = (ftWrenchInRobotFilt.get_force().x() - gprPrediction->mean) / gprPrediction->sigma;

          auto offsetMean = gprPrediction->mean + deviationOffset * gprPrediction->sigma;
          double offsetDeviation = deviation - deviationOffset;

          jsonLogger.addSubfield(logger::MessageType::MODEL, "gpr", "mean", gprPrediction->mean);
          jsonLogger.addSubfield(logger::MessageType::MODEL, "gpr", "sigma", gprPrediction->sigma);
          jsonLogger.addSubfield(logger::MessageType::MODEL, "gpr", "deviation", deviation);
          jsonLogger.addSubfield(logger::MessageType::MODEL, "gpr", "deviation_offset", deviationOffset);
          jsonLogger.addSubfield(logger::MessageType::MODEL, "gpr", "offset_mean", offsetMean);
          jsonLogger.addSubfield(logger::MessageType::MODEL, "gpr", "offset_deviation", offsetDeviation);

          std::cout << "GPR >>> F: " << ftWrenchInRobotFilt.get_force().x() << ", u: " << gprPrediction->mean
                    << ", sigma: " << gprPrediction->sigma << ", deviation: " << deviation;

          std::cout << " | u': " << offsetMean << ", deviation': " << offsetDeviation
                    << ", offset: " << deviationOffset << std::endl;

          // incrementally adapt the offset
          if (abs(deviationOffset + deviationRate * offsetDeviation) < ITS.params["cut"]["max_force_deviation"].as<double>()) {
            deviationOffset += deviationRate * offsetDeviation;
          }

          if (cutStable < 0 && abs(offsetDeviation) < ITS.params["cut"]["max_force_deviation"].as<double>()) {
            ++cutStable;
          }

          if (cutStable >= 0) {
            if (offsetDeviation < maxDeviation[0]) {
              maxDeviation[0] = offsetDeviation;
            } else if (offsetDeviation > maxDeviation[1]) {
              maxDeviation[1] = offsetDeviation;
            }

            if (abs(offsetDeviation) >= ITS.params["cut"]["max_force_deviation"].as<double>()) {
              //TODO: accumulate error and abort if the total exceeds some time / magnitude constraints
              std::cout << "### Cut force outside of expected range! Deviation: " << deviation << std::endl;
              ITS.setRetractionPhase(eeInTask);
              finished = true;
              gpr.stop();
              trialState = RETRACTION;
              std::cout << "### STARTING RETRACTION PHASE" << std::endl;
            }
          }
        }

        double angle = touchPose.get_orientation().angularDistance(eeInRobot.get_orientation()) * 180 / M_PI;
        double distance = (eeInRobot.get_position() - touchPose.get_position()).norm();
        if (angle > ITS.params["cut"]["arc_angle"].as<double>()
            || distance > ITS.params["cut"]["cut_distance"].as<double>()) {
          ITS.setRetractionPhase(eeInTask);
          finished = true;
          gpr.stop();
          std::cout << "Cut phase finished. Min / Max observed deviation: " << maxDeviation[0]
                    << ", " << maxDeviation[1] << std::endl;
          trialState = RETRACTION;
          std::cout << "### STARTING RETRACTION PHASE" << std::endl;
        }
        break;
      }
      case RETRACTION:
        if (!finished && (eeInTask.get_position() - ITS.pointDS.get_attractor().get_position()).norm() < 0.02) {
          // check if there are more touch points to record
          if (!CP.full) {
            auto nextPoint = CP.getNextPointInTask();
            std::cout << "Next touch point in task: " << std::endl;
            std::cout << nextPoint << std::endl;
            ITS.pointDS.set_attractor(nextPoint);
          } else {
            ITS.pointDS.set_attractor(CP.getStart());
          }
          trialState = TrialState::APPROACH;
          std::cout << "### STARTING APPROACH PHASE" << std::endl;
        }
        break;
    }

    CartesianTwist commandTwistInRobot = ITS.getTwistCommand(eeInTask, taskInRobot, trialState);
    CartesianWrench commandWrenchInRobot = ITS.getWrenchCommand(commandTwistInRobot, eeInRobot);

    frankalwi::utils::fromJointTorque(jacobian.transpose() * commandWrenchInRobot, command);
    franka.send(command);

    jsonLogger.addTime();
    jsonLogger.addField(logger::CONTROL, "phase", trialStateMap.at(trialState));
    if (!ITS.cut || CP.full) {
      jsonLogger.addBody(logger::RAW, eeInRobot);
      jsonLogger.addBody(logger::RAW, eeInTask);
      jsonLogger.addBody(logger::RAW, eeLocalTwist);
      jsonLogger.addBody(logger::RAW, taskInRobot);
      jsonLogger.addBody(logger::RAW, ftWrenchInRobot);

      jsonLogger.addBody(logger::FILTERED, eeInRobotFilt);
      jsonLogger.addBody(logger::FILTERED, eeLocalTwistFilt);
      jsonLogger.addBody(logger::FILTERED, ftWrenchInRobotFilt);
      jsonLogger.addCommand(commandTwistInRobot, commandWrenchInRobot);

      std::vector<double> gains(4);
      Eigen::MatrixXd::Map(&gains[0], 4, 1) = ITS.ctrl.get_gains();
      jsonLogger.addField(logger::CONTROL, "gains",  gains);
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

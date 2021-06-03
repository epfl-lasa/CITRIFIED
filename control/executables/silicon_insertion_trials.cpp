#include "controllers/IncisionTrialSystem.h"
#include "sensors/ForceTorqueSensor.h"
#include "franka_lwi/franka_lwi_utils.h"
#include "filters/DigitalButterworth.h"
#include "network/interfaces.h"
#include "logger/JSONLogger.h"
#include "learning/ESNWrapper.h"

using namespace state_representation;

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // set up control system
  auto configFile = std::string(TRIAL_CONFIGURATION_DIR) + "silicon_trials_parameters.yaml";
  IncisionTrialSystem ITS(configFile);

  std::cout << ITS.trialName << std::endl;

  // set up logger
  logger::JSONLogger jsonLogger(ITS.trialName);
  jsonLogger.addMetaData(ITS.trialName, ITS.yamlContent);
  jsonLogger.addSubfield(logger::METADATA, "insertion", "depth", ITS.params["insertion"]["depth"].as<double>());
  jsonLogger.addSubfield(logger::METADATA, "insertion", "pitch", ITS.params["attractor"]["pitch_angle"].as<double>());

  // set up FT sensor
  sensors::ToolSpec tool = {
      .centerOfMass = Eigen::Vector3d(0, 0, 0.02), .mass = 0.07
  };
  sensors::ForceTorqueSensor ft_sensor("ft_sensor", "128.178.145.248", 100, tool);

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
  Eigen::VectorXd esnInputSample(2);
  learning::ESNWrapper esn(esnConfigFile, ITS.esnBufferSize, ITS.esnMinTimeBetweenPredictions);
  esn.setDerivativeCalculationIndices({0, 1});
  std::vector<learning::esnPrediction> esnPredictionCollection;
  learning::esnPrediction finalESNPrediction;
  std::cout << "ESN ready" << std::endl;

  // set up filters
  filter::DigitalButterworth wrenchFilter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 6);
  filter::DigitalButterworth stateFilter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 13);

  // set up robot connection
  network::Interface franka(network::InterfaceType::FRANKA_PAPA_16);
  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  // prepare all state objects
  CartesianState eeInRobot("ee", "robot");
  CartesianState eeInRobotFilt = eeInRobot;
  CartesianWrench ftWrenchInRobot("ft_sensor", "robot");
  CartesianWrench ftWrenchInRobotFilt = ftWrenchInRobot;

  state_representation::Jacobian jacobian("franka", 7, "ee", "robot");

  CartesianPose touchPose = eeInRobot;

  jsonLogger.write();

  std::cout << "Ready to begin trial!" << std::endl;
  // start main control loop
  bool touched = false;
  int tubes = 0;
  bool set_attractor_position_from_ee = true;
  CartesianState taskInRobot("task", "robot");
  int iterations = 0;
  auto frequencyTimer = std::chrono::system_clock::now();
  auto pauseTimer = std::chrono::system_clock::now();
  TrialState trialState = TrialState::APPROACH;
  while (franka.receive(state)) {

    //  update states
    frankalwi::utils::toCartesianState(state, eeInRobot);
    frankalwi::utils::toJacobian(state.jacobian, jacobian);

    if (set_attractor_position_from_ee) {
      taskInRobot.set_position(eeInRobot.get_position() + Eigen::Vector3d(0, 0, 0.0));
      taskInRobot.set_orientation(Eigen::Quaterniond(0, 0, 0, 1));
      ITS.setDSBaseFrame(taskInRobot);
      set_attractor_position_from_ee = false;
    }
    auto eeInTask = taskInRobot.inverse() * eeInRobot;

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

    ftWrenchInRobotFilt.set_wrench(wrenchFilter.computeFilterOutput(ftWrenchInRobot.get_wrench()));

    if (ftWrenchInRobot.get_force().norm() > ITS.params["default"]["max_force"].as<double>()) {
      std::cout << "Max force exceeded" << std::endl;
      ITS.setRetractionPhase(eeInTask);
      trialState = RETRACTION;
      std::cout << "### ENTERING FINAL RETRACTION PHASE" << std::endl;
      break;
    }

    switch (trialState) {
      case APPROACH:
        if ((eeInTask.get_position() - ITS.pointDS.get_attractor().get_position()).norm() < 0.005) {
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
            std::cout << "Starting ESN thread" << std::endl;
            esn.start();
            pauseTimer = std::chrono::system_clock::now();
            trialState = INSERTION;
            std::cout << "### STARTING INSERTION PHASE" << std::endl;
          }
        }
        break;
      }
      case INSERTION: {
        if (abs(ftWrenchInRobotFilt.get_force().z()) > ITS.params["touch"]["touch_force"].as<double>()) {
          touched = true;
        }
        esnSkip--;
        if (esnSkip <= 0) {
          // combine sample for esn input
          esnInputSample << eeInRobotFilt.get_linear_velocity().z(), ftWrenchInRobotFilt.get_force().z();

          esn.addSample(jsonLogger.getTime(), esnInputSample);
          esnSkip = 2;
        }
        Eigen::MatrixXd timeBuffer, dataBuffer;
        auto esnPrediction = esn.getLastPrediction(timeBuffer, dataBuffer);
        if (esnPrediction) {
          if (esnPrediction->classIndex == 1) {
            ++tubes;
          } else {
            tubes = 0;
          }
          std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - pauseTimer;
          if (tubes > 3 && elapsed_seconds.count() > 2.0f && (ITS.pointDS.get_base_frame().get_position() - eeInRobot.get_position()).norm() > 0.002) {
            ITS.setRetractionPhase(eeInTask, 0.05);
            trialState = RETRACTION;
            std::cout << "### STARTING RETRACTION PHASE" << std::endl;
          }

          std::cout << esnPrediction->className << std::endl;
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
                                 "acceleration_z",
                                 std::vector<double>(dataBuffer.col(0).data(),
                                                     dataBuffer.col(0).data() + ITS.esnBufferSize));
          jsonLogger.addSubfield(logger::MessageType::ESN,
                                 "input",
                                 "force_derivative_z",
                                 std::vector<double>(dataBuffer.col(1).data(),
                                                     dataBuffer.col(1).data() + ITS.esnBufferSize));
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
    jsonLogger.addField(logger::CONTROL, "phase", trialStateMap.at(trialState));
    jsonLogger.addBody(logger::RAW, eeInRobot);
    jsonLogger.addBody(logger::RAW, eeInTask);
    jsonLogger.addBody(logger::RAW, taskInRobot);
    jsonLogger.addBody(logger::RAW, ftWrenchInRobot);

    jsonLogger.addBody(logger::FILTERED, eeInRobotFilt);
    jsonLogger.addBody(logger::FILTERED, ftWrenchInRobotFilt);
    jsonLogger.addCommand(commandTwistInRobot, commandWrenchInRobot);

    std::vector<double> gains(4);
    Eigen::MatrixXd::Map(&gains[0], 4, 1) = ITS.ctrl.get_gains();
    jsonLogger.addField(logger::CONTROL, "gains", gains);

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

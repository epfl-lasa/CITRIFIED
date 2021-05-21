#include <yaml-cpp/yaml.h>

#include <dynamical_systems/Linear.hpp>
#include <dynamical_systems/Circular.hpp>

#include "controllers/FrankaController.h"
#include "controllers/IncisionTrialSystem.h"
#include "controllers/CutProber.h"

#include "franka_lwi/franka_lwi_utils.h"
#include "sensors/ForceTorqueSensor.h"
#include "filters/DigitalButterworth.h"
#include "network/interfaces.h"
#include "logger/JSONLogger.h"
#include "learning/ESNWrapper.h"
#include "learning/GPR.h"

using namespace state_representation;

class QuebecWrapper {
public:
  explicit QuebecWrapper(const YAML::Node& params) :
      franka_quebec(network::InterfaceType::FRANKA_QUEBEC_17, "quebec", "task"),
      frame_quebec(CartesianState::Identity("quebec", "papa")),
      task_in_quebec("task", "quebec"),
      orientation_ds_quebec(task_in_quebec),
      position_ds_quebec(task_in_quebec, 1, 1, 1),
      ctrl_quebec(100, 100, 4, 4) {
    // assume frame papa = world
    frame_quebec.set_position(0.899, 0, 0);
    frame_quebec.set_orientation(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())));

    std::vector<double> orientation_gains = {0.0, 0.0, 0.0, 10.0, 10.0, 10.0};
    state_representation::CartesianPose attractor_quebec("attractor_quebec",frame_quebec.get_name());
    attractor_quebec.from_std_vector(params["quebec"]["position"].as<std::vector<double>>());
    attractor_quebec.from_std_vector(params["quebec"]["orientation"].as<std::vector<double>>());
    orientation_ds_quebec = dynamical_systems::Linear<CartesianState>(attractor_quebec, orientation_gains);

    position_ds_quebec.set_center(attractor_quebec);
    position_ds_quebec.set_radiuses(params["quebec"]["ellipse"].as<std::vector<double>>());
    position_ds_quebec.set_normal_gain(params["quebec"]["normal_gain"].as<double>());
    position_ds_quebec.set_planar_gain(params["quebec"]["planar_gain"].as<double>());
    position_ds_quebec.set_circular_velocity(params["quebec"]["circular_velocity"].as<double>());

    auto gains = params["quebec"]["ctrl_gains"].as<std::vector<double>>();
    ctrl_quebec.set_gains(Eigen::Vector4d(gains.data()));

    franka_quebec.set_callback([this] (const CartesianState& state, const Jacobian& jacobian) -> JointTorques {
      return control_loop_quebec(state, jacobian);
    });
  }

  JointTorques control_loop_quebec(const CartesianState& state, const Jacobian& jacobian) {
    task_in_quebec = state;
    CartesianTwist dsTwist = orientation_ds_quebec.evaluate(state) + position_ds_quebec.evaluate(state);
    dsTwist.clamp(0.5, 0.75);

    return ctrl_quebec.compute_command(dsTwist, state, jacobian);
  }

  controllers::FrankaController franka_quebec;
  CartesianState frame_quebec;
  CartesianState task_in_quebec;
  dynamical_systems::Linear<CartesianState> orientation_ds_quebec;
  dynamical_systems::Circular position_ds_quebec;
  controllers::impedance::CartesianTwistController ctrl_quebec;
};

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(3);

  // set up control system
  auto configFile = std::string(TRIAL_CONFIGURATION_DIR) + "dual_incision_trials_parameters.yaml";
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
  jsonLogger.write();

  // set up FT sensor
  sensors::ToolSpec tool = {
      .centerOfMass = Eigen::Vector3d(0, 0, 0.02),
      .mass = 0.07
  };
  sensors::ForceTorqueSensor ft_sensor("ft_sensor", "128.178.145.248", 100, tool);

  // set up GPR predictor
  std::cout << "Waiting for GPR server..." << std::endl;
  learning::GPR<2> gpr;
  bool gprStarted = false;
  gpr.start(1);
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

  // set up robots
  network::Interface franka_papa(network::InterfaceType::FRANKA_PAPA_16);
  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  QuebecWrapper quebec_wrapper(ITS.params);
  quebec_wrapper.franka_quebec.start();

  // prepare all state objects
  CartesianState eeInPapa("ee", "papa");
  CartesianState eeInPapaFilt = eeInPapa;
  CartesianWrench ftWrenchInPapa("ft_sensor", "papa");
  CartesianTwist eeLocalTwist("ee", "ee");
  CartesianTwist eeLocalTwistFilt = eeLocalTwist;
  CartesianWrench ftWrenchInPapaFilt = ftWrenchInPapa;

  state_representation::Jacobian jacobian("papa", 7, "ee", "papa");

  CartesianPose touchPose = eeInPapa;
  bool touchPoseSet = false;

  filter::DigitalButterworth filter("esn_filter", std::string(TRIAL_CONFIGURATION_DIR) + "filter_config.yaml", 4);
  auto filterInput = Eigen::VectorXd::Zero(4);


  std::cout << "Ready to begin trial!" << std::endl;
  // start main control loop
  bool finished = false;
  int iterations = 0;
  auto frequencyTimer = std::chrono::system_clock::now();
  auto pauseTimer = std::chrono::system_clock::now();
  TrialState trialState = TrialState::APPROACH;
  while (franka_papa.receive(state)) {

    //  update states
    frankalwi::utils::toCartesianState(state, eeInPapa);
    frankalwi::utils::toJacobian(state.jacobian, jacobian);
    eeLocalTwist.set_twist((-1.0 * CartesianTwist(eeInPapa).inverse()).get_twist());

    // update target states
    CartesianState taskInPapa = eeInPapa;
    if (!quebec_wrapper.task_in_quebec.is_empty()) {
      taskInPapa = quebec_wrapper.frame_quebec * quebec_wrapper.task_in_quebec;
    }
    auto eeInTask = taskInPapa.inverse() * eeInPapa;
    ITS.setDSBaseFrame(taskInPapa);

    // update ft wrench
    if (ft_sensor.biasOK()) {
      ft_sensor.readContactWrench(ftWrenchInPapa, eeInPapa.get_orientation().toRotationMatrix());
    }

    // filter data: robot state
    Eigen::Matrix<double, 13, 1> stateVector;
    stateVector << eeInPapa.get_pose(), eeInPapa.get_twist();
    auto stateFilt = stateFilter.computeFilterOutput(stateVector);
    eeInPapaFilt.set_pose(stateFilt.head(7));
    eeInPapaFilt.set_twist(stateFilt.tail(6));

    // filter data: localised twist and force
    eeLocalTwistFilt.set_twist(twistFilter.computeFilterOutput(eeLocalTwist.get_twist()));
    ftWrenchInPapaFilt.set_wrench(wrenchFilter.computeFilterOutput(ftWrenchInPapa.get_wrench()));

    if (ftWrenchInPapa.get_force().norm() > ITS.params["default"]["max_force"].as<double>()) {
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
          if (ft_sensor.computeBias(eeInPapa.get_orientation().toRotationMatrix(), 2000)) {
            ITS.setTouchPhase();
            trialState = TOUCH;
            std::cout << "### STARTING TOUCH PHASE" << std::endl;
          }
        }
        break;
      }
      case TOUCH:
        if (abs(ftWrenchInPapaFilt.get_force().z()) > ITS.params["touch"]["touch_force"].as<double>()) {
          std::cout << "Surface detected at position " << eeInPapa.get_position().transpose() << std::endl;

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

            touchPose = eeInPapa;
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
          depth = (touchPose.get_position() - eeInPapa.get_position()).norm();
        }
        jsonLogger.addField(logger::MODEL, "depth", depth);
        if (depth > ITS.params["insertion"]["depth"].as<double>()) {
          ITS.zVelocity = 0;

          std::cout << "Stopping ESN thread" << std::endl;
          esn.stop();

          // hold the current position
          ITS.setRetractionPhase(eeInTask, ITS.params["insertion"]["depth"].as<double>() - ITS.params["cut"]["depth"].as<double>());
          pauseTimer = std::chrono::system_clock::now();
          trialState = CLASSIFICATION;
          std::cout << "### CLASSIFYING - INCISION DEPTH REACHED" << std::endl;
        }

        esnSkip--;
        if (esnSkip <= 0) {
          // combine sample for esn input
          esnInputSample << depth,
              eeLocalTwistFilt.get_linear_velocity().x(),
              eeLocalTwistFilt.get_linear_velocity().z(),
              ftWrenchInPapaFilt.get_force().x(),
              ftWrenchInPapaFilt.get_force().z();

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
        std::cout << "### PAUSING - TISSUE CLASSIFIED" << std::endl;
        std::cout << finalESNPrediction.className << std::endl;
        break;
      }
      case PAUSE: {
        if (!gprStarted) {
          gprStarted = gpr.start(1);
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
        jsonLogger.addField(logger::MODEL, "depth", position.z() - eeInTask.get_position().z());
        position.z() -= ITS.params["cut"]["depth"].as<double>();
        center.set_position(position);
        ITS.ringDS.set_center(center);

        std::array<double, 2>
            gprRequest = {position.z() - eeInTask.get_position().z(), eeInPapaFilt.get_linear_velocity().x()};
        gpr.updateState(gprRequest);
        if (auto gprPrediction = gpr.getLastPrediction()) {
          jsonLogger.addField(logger::MODEL, "gpr", gprPrediction->data());
        }

        double angle = touchPose.get_orientation().angularDistance(eeInPapa.get_orientation()) * 180 / M_PI;
        double distance = eeInTask.dist(CP.getTouchPointInTask(0), CartesianStateVariable::POSITION);
        if (angle > ITS.params["cut"]["arc_angle"].as<double>()
            || distance > ITS.params["cut"]["cut_distance"].as<double>()) {
          ITS.setRetractionPhase(eeInTask);
          finished = true;
          gpr.stop();
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

    CartesianTwist commandTwistInPapa = ITS.getTwistCommand(eeInTask, taskInPapa, trialState);
    CartesianWrench commandWrenchInPapa = ITS.getWrenchCommand(commandTwistInPapa, eeInPapa);

    frankalwi::utils::fromJointTorque(jacobian.transpose() * commandWrenchInPapa, command);
    franka_papa.send(command);

    jsonLogger.addTime();
    jsonLogger.addField(logger::CONTROL, "phase", trialStateMap.at(trialState));
    if (!ITS.cut || CP.full) {
      jsonLogger.addBody(logger::RAW, eeInPapa);
      jsonLogger.addBody(logger::RAW, eeInTask);
      jsonLogger.addBody(logger::RAW, eeLocalTwist);
      jsonLogger.addBody(logger::RAW, taskInPapa);
      jsonLogger.addBody(logger::RAW, ftWrenchInPapa);

      jsonLogger.addBody(logger::FILTERED, eeInPapaFilt);
      jsonLogger.addBody(logger::FILTERED, eeLocalTwistFilt);
      jsonLogger.addBody(logger::FILTERED, ftWrenchInPapaFilt);
      jsonLogger.addCommand(commandTwistInPapa, commandWrenchInPapa);

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

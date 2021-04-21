#pragma once

#include <yaml-cpp/yaml.h>

#include <dynamical_systems/Linear.hpp>
#include <dynamical_systems/Ring.hpp>

#include "controllers/TwistController.h"

enum TrialState {
  APPROACH, CALIBRATION, TOUCH, INSERTION, PAUSE, CUT, RETRACTION
};

static const std::map<TrialState, std::string> trialStateMap
    {{APPROACH, "approach"},
     {CALIBRATION, "calibration"},
     {TOUCH, "touch"},
     {INSERTION, "insertion"},
     {PAUSE, "pause"},
     {CUT, "cut"},
     {RETRACTION, "retraction"}};

class IncisionTrialSystem {
public:
  explicit IncisionTrialSystem(const std::string& configFile);

  state_representation::CartesianTwist getTwistCommand(const state_representation::CartesianState& eeInTask,
                                                       const state_representation::CartesianState& taskInRobot,
                                                       TrialState trialState) const;

  state_representation::CartesianWrench getWrenchCommand(const state_representation::CartesianTwist& twistInRobot,
                                                         const state_representation::CartesianState& eeInRobot);

  void setDSBaseFrame(const state_representation::CartesianState& base);

  void setTouchPhase();

  void setInsertionPhase();

  void setCutPhase(const state_representation::CartesianPose& eeInTask);

  void setRetractionPhase(const state_representation::CartesianPose& eeInTask, double offset = 0.1);

  // properties
  YAML::Node params;
  std::string trialName;
  std::string yamlContent;

  dynamical_systems::Linear<state_representation::CartesianState> pointDS;
  dynamical_systems::Ring ringDS;
  controllers::TwistController ctrl;
  std::vector<double> DSgains;

  double zVelocity = 0.0;
  bool cut = false;
};

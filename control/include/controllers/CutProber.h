#pragma once

#include "controllers/IncisionTrialSystem.h"

class CutProber : public IncisionTrialSystem {
public:
  explicit CutProber(const std::string& configFile);

  void reset();

  // starting point for the forward integration
  void setStart(const state_representation::CartesianPose& eeInTask);

  // integrate the position to find and return the next probe point
  state_representation::CartesianPose getNextPointInTask();

  // add a surface height sample at an XY position in the task frame
  void addPoint(const state_representation::CartesianPose& eeInTask);

  // get a surface height estimate for an XY position in the task frame
  double estimateHeightInTask(const state_representation::CartesianPose& eeInTask) const;

  state_representation::CartesianPose getStart() const;

  int count() const;
  double dist() const;
  double angle() const;

  int stepCount = 100;
  bool full = false;

private:
  void step();

  state_representation::CartesianPose startPose_;
  state_representation::CartesianPose poseInTask_;
  Eigen::Matrix2Xd xy_;
  Eigen::VectorXd z_;
  std::chrono::milliseconds dt_;
  double dist_ = 0;
  double ang_ = 0;
  double tol_ = 1e-5;
};
#include "controllers/CutProber.h"

#include <numeric>

using namespace state_representation;

CutProber::CutProber(const std::string& configFile) : IncisionTrialSystem(configFile), dt_(1) {
  stepCount = params["probe"]["ms_per_step"].as<int>();
}

void CutProber::reset() {
  xy_ = Eigen::MatrixX2d::Zero(2, 0);
  z_ = Eigen::VectorXd::Zero(0);
  dist_ = 0;
  ang_ = 0;
  full = false;
}

// starting point for the forward integration
void CutProber::setStart(const CartesianPose& eeInTask) {
  startPose_ = eeInTask;
  poseInTask_ = eeInTask;
  setCutPhase(poseInTask_);
  zVelocity = 0;
}

// integrate the position to find and return the next probe point
CartesianPose CutProber::getNextPointInTask() {
  for (int ms = 0; ms < stepCount; ++ms) {
    step();
  }
  return poseInTask_;
}

// add a surface height sample at an XY position in the task frame
void CutProber::addPoint(const CartesianPose& eeInTask) {
  xy_.conservativeResize(Eigen::NoChange, xy_.cols() + 1);
  xy_.col(xy_.cols() - 1) = Eigen::Vector2d(eeInTask.get_position().x(), eeInTask.get_position().y());

  z_.conservativeResize(z_.rows() + 1, Eigen::NoChange);
  z_(z_.size() - 1) = eeInTask.get_position().z();
}

// get a surface height estimate for an XY position in the task frame
double CutProber::estimateHeightInTask(const CartesianPose& eeInTask) const {
  if (xy_.cols() == 0) {
    return 0;
  }

  Eigen::Matrix2Xd diffs = (xy_.colwise() - Eigen::Vector2d(eeInTask.get_position().x(), eeInTask.get_position().y())).matrix();
  Eigen::VectorXd dist = diffs.colwise().norm();

  // get the index of the closest point
  Eigen::VectorXd::Index minIndex;
  double min = dist.minCoeff(&minIndex);
  if (abs(min) < tol_ || xy_.cols() == 1) {
    return z_(minIndex);
  }

  // the current position is between the closest sample and one other sample,
  // which is either before or after the closest sample
  Eigen::VectorXd::Index other;
  if (minIndex == 0) {
    other = 1;
  } else if (minIndex == (xy_.cols() - 1)) {
    other = xy_.cols() - 2;
  } else {
    double a = diffs.col(minIndex).normalized().dot((xy_.col(minIndex) - xy_.col(minIndex - 1)).normalized());
    double b = diffs.col(minIndex).normalized().dot((xy_.col(minIndex) - xy_.col(minIndex + 1)).normalized());
    other = a > b ? minIndex - 1 : minIndex + 1;
  }

  // find the height as the linearly weighted average between the closest two sample points
  double weight = dist(minIndex) / (dist(minIndex) + dist(other));
  return z_(minIndex) * (1 - weight) + z_(other) * weight;
}

CartesianPose CutProber::getStart() const {
  return startPose_;
}

int CutProber::count() const {
  return z_.size();
}

double CutProber::dist() const {
  return dist_;
}

double CutProber::angle() const {
  return ang_;
}

void CutProber::step() {
  CartesianTwist twistInTask = ringDS.evaluate(poseInTask_);
  auto diff = dt_ * twistInTask;
  dist_ += diff.get_position().norm();
  ang_ += diff.get_orientation().angularDistance(Eigen::Quaterniond::Identity());
  poseInTask_ = dt_ * twistInTask + poseInTask_;
}
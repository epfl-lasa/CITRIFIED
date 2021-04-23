#include "controllers/CutProber.h"

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
  Eigen::VectorXd dist =
      (xy_.colwise() - Eigen::Vector2d(eeInTask.get_position().x(), eeInTask.get_position().y())).colwise().norm();
  // TODO: if any dist is < tol, then just return the z value of that index. Otherwise, calculate weighted average

  Eigen::VectorXd weights = dist.cwiseInverse().cwiseAbs2();
  weights /= weights.sum();

  return weights.dot(z_);
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
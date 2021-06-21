#pragma once

#include <mutex>
#include <memory>

#include <mathlib/MathLib.h>
#include <fast_gmm/CDDynamics.h>
#include <lagsDS.h>

namespace motion_generator {

class LagsDSMotionGenerator {

public:
  LagsDSMotionGenerator(double frequency,
                        int k,
                        int m,
                        std::vector<double> priors,
                        std::vector<double> mu,
                        std::vector<double> sigma,
                        std::vector<double> a_g,
                        std::vector<double> att_g,
                        std::vector<double> a_l,
                        std::vector<double> a_d,
                        std::vector<double> att_l,
                        std::vector<double> w_l,
                        std::vector<double> b_l,
                        double scale,
                        double b_g,
                        string gpr_path);

  ~LagsDSMotionGenerator() = default;

  bool init();

  Eigen::VectorXd computeDesiredVelocity(const Eigen::VectorXd& input);

private:

  bool initializeDS();

  // DS variables
  double dt_;
  double max_desired_vel_;

  // Paramaters for LAGS Class
  int k_;
  int m_;
  std::vector<double> priors_;
  std::vector<double> mu_;
  std::vector<double> sigma_;
  std::vector<double> a_g_;
  std::vector<double> att_g_;
  std::vector<double> a_l_;
  std::vector<double> a_d_;
  std::vector<double> att_l_;
  std::vector<double> w_l_;
  std::vector<double> b_l_;
  double scale_;
  double b_g_;
  string gpr_path_;

  // Instantiate LAGS Class
  std::unique_ptr<lagsDS> LAGS_DS_;

  // Filter variables
  std::unique_ptr<CDDynamics> CCDyn_filter_;

  double Wn_;
  MathLib::Vector accLimits_;
  MathLib::Vector velLimits_;

  // Class variables
  std::mutex mutex_;

  bool b_global_;

  Eigen::VectorXd learned_att_;
  Eigen::VectorXd target_pose_;

  double scaling_factor_;
  double ds_vel_limit_;
};

}
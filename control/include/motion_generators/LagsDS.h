#pragma once

#include <mutex>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Core>

#include <fast_gmm/CDDynamics.h>
#include <lagsDS.h>

namespace motion_generator {

class LagsDS {

public:
  LagsDS(double frequency,
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
         std::string gpr_path);

  ~LagsDS() = default;

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
  std::string gpr_path_;

  // Instantiate LAGS Class
  std::unique_ptr<lagsDS> LAGS_DS_;

  // Filter variables
  std::unique_ptr<fast_gmm::CDDynamics> CCDyn_filter_;

  double Wn_;
  Eigen::VectorXd accLimits_;
  Eigen::VectorXd velLimits_;

  // Class variables
  std::mutex mutex_;

  bool b_global_;

  Eigen::VectorXd learned_att_;
  Eigen::VectorXd target_pose_;

  double scaling_factor_;
  double ds_vel_limit_;
};

}
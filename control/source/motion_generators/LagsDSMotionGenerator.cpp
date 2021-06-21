#include "motion_generators/LagsDSMotionGenerator.h"

namespace motion_generator {

LagsDSMotionGenerator::LagsDSMotionGenerator(double frequency,
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
                                             std::string gpr_path) :
    k_(k),
    m_(m),
    priors_(std::move(priors)),
    mu_(std::move(mu)),
    sigma_(std::move(sigma)),
    a_g_(std::move(a_g)),
    att_g_(std::move(att_g)),
    a_l_(std::move(a_l)),
    a_d_(std::move(a_d)),
    att_l_(std::move(att_l)),
    w_l_(std::move(w_l)),
    b_l_(std::move(b_l)),
    scale_(scale),
    b_g_(b_g),
    gpr_path_(std::move(gpr_path)),
    dt_(1 / frequency),
    Wn_(0),
    scaling_factor_(0.5),
    ds_vel_limit_(0.5),
    b_global_(false) {}

bool LagsDSMotionGenerator::init() {
  if (!initializeDS()) {
    std::cerr << "ERROR initializing the DS" << std::endl;
    return false;
  } else {
    return true;
  }
}

bool LagsDSMotionGenerator::initializeDS() {

  // Checking Size of Variables
  if (priors_.size() != k_) {
    std::cerr << "initializeDS: " << k_ << " priors are expected while " << priors_.size() << " are provided."
              << std::endl;
  }

  if (mu_.size() != k_ * m_) {
    std::cerr << "initializeDS: " << k_ * m_ << " elements in Mu are expected while " << mu_.size() << " are provided."
              << std::endl;
  }

  if (sigma_.size() != k_ * m_ * m_) {
    std::cerr << "initializeDS: " << k_ * m_ * m_ << " elements in Sigma are expected while " << sigma_.size()
              << " are provided." << std::endl;;
    return false;
  }

  if (a_g_.size() != k_ * m_ * m_) {
    std::cerr << "initializeDS: " << k_ * m_ * m_ << " elements in A_g are expected while " << a_g_.size()
              << " are provided." << std::endl;;
    return false;
  }

  if (att_g_.size() != m_) {
    std::cerr << "initializeDS: " << m_ << " elements in att_g are expected while " << att_g_.size() << " are provided."
              << std::endl;;
    return false;
  }

  if (a_l_.size() != k_ * m_ * m_) {
    std::cerr << "initializeDS: " << k_ * m_ * m_ << " elements in A_l are expected while " << a_l_.size()
              << " are provided." << std::endl;;
    return false;
  }

  if (a_d_.size() != k_ * m_ * m_) {
    std::cerr << "initializeDS: " << k_ * m_ * m_ << " elements in A_d are expected while " << a_d_.size()
              << " are provided." << std::endl;;
    return false;
  }

  if (att_l_.size() != k_ * m_) {
    std::cerr << "initializeDS: " << k_ * m_ << " elements in att_l are expected while " << att_l_.size()
              << " are provided." << std::endl;;
    return false;
  }

  if (w_l_.size() != k_ * m_) {
    std::cerr << "initializeDS: " << k_ * m_ << " elements in w_l are expected while " << w_l_.size()
              << " are provided." << std::endl;;
    return false;
  }

  if (b_l_.size() != k_) {
    std::cerr << "initializeDS: " << k_ << " elements in b_l are expected while " << b_l_.size() << " are provided."
              << std::endl;
  }

  // Instantiate lags-DS Model with parameters read from Yaml file
  LAGS_DS_ = std::make_unique<lagsDS>(k_,
                                      m_,
                                      priors_,
                                      mu_,
                                      sigma_,
                                      a_g_,
                                      att_g_,
                                      a_l_,
                                      a_d_,
                                      att_l_,
                                      w_l_,
                                      b_l_,
                                      scale_,
                                      b_g_,
                                      gpr_path_);
  target_pose_.resize(m_);
  learned_att_.resize(m_);

  for (int i = 0; i < att_g_.size(); i++) {
    target_pose_(i) = att_g_[i];
    learned_att_(i) = att_g_[i];
  }

  // Set attractor, for now it is assumed to be static and the same as the learned model
  LAGS_DS_->set_att_g(learned_att_);

  // initializing the filter
  CCDyn_filter_ = std::make_unique<CDDynamics>(m_, dt_, Wn_);

  // we should set the size automatically
  velLimits_.Resize(m_);
  CCDyn_filter_->SetVelocityLimits(velLimits_);
  accLimits_.Resize(m_);
  CCDyn_filter_->SetAccelLimits(accLimits_);

  MathLib::Vector initial(m_);
  initial.Zero();
  CCDyn_filter_->SetState(initial);
  CCDyn_filter_->SetTarget(initial);
  return true;
}

Eigen::VectorXd LagsDSMotionGenerator::computeDesiredVelocity(const Eigen::VectorXd& input) {
  mutex_.lock();
  Eigen::VectorXd desired_velocity, desired_velocity_filtered;
  if (b_global_) {
    // If you only want to use the global component
    desired_velocity = LAGS_DS_->compute_fg(input - (target_pose_ - learned_att_), learned_att_);
  } else {
    // If you only want to use the full combined Locally Active Globally Stable DS
    desired_velocity = LAGS_DS_->compute_f(input - (target_pose_ - learned_att_), 1);
  }

  if (std::isnan(desired_velocity.lpNorm<2>())) {
    std::cerr << "DS is generating NaN. Setting the output to zero." << std::endl;
    desired_velocity.setZero();
  }

  desired_velocity = desired_velocity * scaling_factor_;

  // Same velocity throughout state-space to avoid jittering
  desired_velocity = desired_velocity / desired_velocity.norm() * 0.05;

  if (desired_velocity.norm() > ds_vel_limit_) {
    std::cerr << "HIGH vel." << std::endl;
    desired_velocity = desired_velocity / desired_velocity.norm() * ds_vel_limit_;
  }

  auto pos_error = (input - target_pose_).lpNorm<2>();
  std::cerr << "Distance to attractor:" << pos_error << std::endl;
  if (pos_error < 1e-4) {
    std::cerr << "[Attractor REACHED] Distance to attractor:" << pos_error << std::endl;
  }

  MathLib::Vector velocity = MathLib::Vector(m_);
  MathLib::Vector velocity_filtered = MathLib::Vector(m_);
  for (int i = 0; i < m_; ++i) {
    velocity[i] = desired_velocity(i);
  }

  CCDyn_filter_->SetTarget(velocity);
  CCDyn_filter_->Update();
  CCDyn_filter_->GetState(velocity_filtered);

  desired_velocity_filtered.resize(m_);
  for (int i = 0; i < m_; ++i) {
    desired_velocity_filtered[i] = velocity_filtered(i);
  }
//  std::cout << desired_velocity.transpose() << std::endl;
//  std::cout << desired_velocity_filtered.transpose() << std::endl;
  mutex_.unlock();
  return desired_velocity;
}
}

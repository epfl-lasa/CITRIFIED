#include <gtest/gtest.h>

#include "controllers/CutProber.h"

class TestCutProber : public ::testing::Test {
protected:
  TestCutProber() :
    CP(std::string(CONTROLLERS_TEST_FIXTURES) + "incision_trial_parameters.yaml"),
    pose(state_representation::CartesianPose::Identity("pose")),
    xy_samples(2, 10), z_samples(10) {
      set_samples();
  }

  void set_samples() {
    xy_samples << 0.0141, 0.0091, 0.0042, -0.0010, -0.0058, -0.0108, -0.0159, -0.0209, -0.0261, -0.0310,
      0.0302, 0.0300, 0.0299, 0.0298, 0.0297, 0.0295, 0.0294, 0.0294, 0.0292, 0.0289;
    z_samples << 0.0519, 0.0551, 0.0585, 0.0607, 0.0626, 0.0629, 0.0621, 0.0610, 0.0590, 0.0559;
  }

  Eigen::Matrix2Xd xy_samples;
  Eigen::VectorXd z_samples;
  state_representation::CartesianPose pose;
  CutProber CP;
};

TEST_F(TestCutProber, HeightEstimateInterpolation) {
  // populate the surface samples
  for (std::size_t sample = 0; sample < xy_samples.cols(); ++sample) {
    pose.set_position(xy_samples(0, sample), xy_samples(1, sample), z_samples(sample));
    CP.addPoint(pose);
  }

  // check the sampled values at and between each sample
  for (std::size_t sample = 0; sample < xy_samples.cols() - 1; ++sample) {
    pose.set_position(xy_samples(0, sample), xy_samples(1, sample), 0);
    EXPECT_NEAR(CP.estimateHeightInTask(pose), z_samples(sample), 1e-5);

    for (int interp = 0; interp < 11; ++interp) {
      double d = static_cast<double>(interp) / 10;
      Eigen::Vector2d xy = xy_samples.col(sample) * (1 - d) + xy_samples.col(sample + 1) * d;
      pose.set_position(xy(0), xy(1), 0);
      EXPECT_NEAR(CP.estimateHeightInTask(pose), z_samples(sample) * (1 - d) + z_samples(sample + 1) * d, 1e-5);
    }
  }
}
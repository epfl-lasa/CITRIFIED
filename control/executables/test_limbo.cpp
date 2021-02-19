#include <fstream>
#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/matern_three_halves.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/mean/null_function.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>

#include <limbo/serialize/text_archive.hpp>

// this tutorials shows how to use a Gaussian process for regression

using namespace limbo;

struct Params {
  struct kernel : public defaults::kernel {

  };
  struct kernel_maternthreehalves : public defaults::kernel_maternthreehalves {
  };
  struct kernel_squared_exp_ard {
    BO_PARAM(double, sigma_sq, 1.0);
    BO_PARAM(int, k, 970);
  };
};

int main(int argc, char** argv) {

//  // our data (1-D inputs, 1-D outputs)
//  std::vector<Eigen::VectorXd> samples;
//  std::vector<Eigen::VectorXd> observations;
//
//  size_t N = 8;
//  for (size_t i = 0; i < N; i++) {
//    Eigen::VectorXd s = tools::random_vector(1).array() * 4.0 - 2.0;
//    samples.push_back(s);
//    observations.push_back(tools::make_vector(std::cos(s(0))));
//  }
//
  // the type of the GP
  using Kernel_t = kernel::MaternThreeHalves<Params>;
  using Mean_t = mean::Data<Params>;
  using GP_t = model::GP<Params, Kernel_t, Mean_t>;

  // 1-D inputs, 1-D outputs
  GP_t gp(1, 1);
//
//  // compute the GP
//  gp.compute(samples, observations);
//
//  // write the predicted data in a file (e.g. to be plotted)
//  std::ofstream ofs("gp.dat");
//  for (int i = 0; i < 100; ++i) {
//    Eigen::VectorXd v = tools::make_vector(i / 100.0).array() * 4.0 - 2.0;
//    Eigen::VectorXd mu;
//    double sigma;
//    std::tie(mu, sigma) = gp.query(v);
//    // an alternative (slower) is to query mu and sigma separately:
//    //  double mu = gp.mu(v)[0]; // mu() returns a 1-D vector
//    //  double s2 = gp.sigma(v);
//    ofs << v.transpose() << " " << mu[0] << " " << sqrt(sigma) << std::endl;
//  }
//
//  // an alternative is to optimize the hyper-parameters
//  // in that case, we need a kernel with hyper-parameters that are designed to be optimized
  using Kernel2_t = kernel::SquaredExpARD<Params>;
  using MeanZ_t = mean::NullFunction<Params>;
  using GP2_t = model::GP<Params, Kernel2_t, MeanZ_t, model::gp::KernelLFOpt<Params>>;
//
  GP2_t gp_ard(1, 1);
  std::cout << gp_ard.kernel_function().params_size() << std::endl;
//  std::cout << gp_ard.kernel_function().params().transpose() << std::endl;
//  Eigen::VectorXd p = gp_ard.kernel_function().params();
//  p[0] = 0.008;
//  p[972 - 1] = 1.0;
//  gp_ard.kernel_function().set_params(p);
//  std::cout << gp_ard.kernel_function().params().transpose() << std::endl;


//  // do not forget to call the optimization!
//  gp_ard.compute(samples, observations, false);
//  gp_ard.optimize_hyperparams();
//
//  // write the predicted data in a file (e.g. to be plotted)
//  std::ofstream ofs_ard("gp_ard.dat");
//  for (int i = 0; i < 100; ++i) {
//    Eigen::VectorXd v = tools::make_vector(i / 100.0).array() * 4.0 - 2.0;
//    Eigen::VectorXd mu;
//    double sigma;
//    std::tie(mu, sigma) = gp_ard.query(v);
//    ofs_ard << v.transpose() << " " << mu[0] << " " << sqrt(sigma) << std::endl;
//  }
//
//  // write the data to a file (useful for plotting)
//  std::ofstream ofs_data("data.dat");
//  for (size_t i = 0; i < samples.size(); ++i) {
//    ofs_data << samples[i].transpose() << " " << observations[i].transpose() << std::endl;
//  }

  auto start = std::chrono::system_clock::now();
  gp_ard.load<serialize::TextArchive>("/tmp/testGP5", false);
  std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
  std::cout << elapsed_seconds.count() << std::endl;
  std::cout << gp_ard.samples().size() << std::endl;
//  std::cout << gp_ard.kernel_function().params().transpose() << std::endl;

//  std::cout << gp.kernel_function().h_params() << std::endl;

  // Sometimes is useful to save an optimized GP
  gp_ard.save<serialize::TextArchive>("/tmp/myGP");

  start = std::chrono::system_clock::now();
  std::ofstream outputFile;
  outputFile.open("/tmp/myGP/test.csv", std::ofstream::out | std::ofstream::trunc);
  for (const auto& sample : gp_ard.samples()) {
    Eigen::VectorXd mu;
    double sigma;
    std::tie(mu, sigma) = gp_ard.query(sample);
    outputFile << mu << ", " << sigma << std::endl;
  }
  outputFile.close();
  elapsed_seconds = std::chrono::system_clock::now() - start;
  std::cout << elapsed_seconds.count() << std::endl;
  std::cout << elapsed_seconds.count() / gp_ard.samples().size() << std::endl;

  return 0;
}

//#include <iostream>
//#include <cstdio>
//
//#include <franka_lwi/franka_lwi_communication_protocol.h>
//#include "network/netutils.h"
//
//void throttledPrintState(frankalwi::proto::StateMessage<7> state, int skip, double avg_freq) {
//  static int count = 0;
//  if (count > skip) {
//    printf("Average frequency of state messages: % 3.3f\n", avg_freq);
//
//    std::cout << "Joints --------------" << std::endl;
//    printf("Joint positions: % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f\n", state.jointPosition[0],
//           state.jointPosition[1], state.jointPosition[2], state.jointPosition[3], state.jointPosition[4],
//           state.jointPosition[5], state.jointPosition[6]);
//
//    std::cout << "Joints --------------" << std::endl;
//    printf("Joint velocities: % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f\n", state.jointVelocity[0],
//           state.jointVelocity[1], state.jointVelocity[2], state.jointVelocity[3], state.jointVelocity[4],
//           state.jointVelocity[5], state.jointVelocity[6]);
//
//    std::cout << "Joints --------------" << std::endl;
//    printf("Joint torques: % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f\n", state.jointTorque[0],
//           state.jointTorque[1], state.jointTorque[2], state.jointTorque[3], state.jointTorque[4], state.jointTorque[5],
//           state.jointTorque[6]);
//
//    std::cout << "STATE --------------" << std::endl;
//    printf("State position xyz:     % 3.3f, % 3.3f, % 3.3f\n", state.eePose.position.x, state.eePose.position.y,
//           state.eePose.position.z);
//    printf("State orientation wxyz: % 3.3f, % 3.3f, % 3.3f, % 3.3f\n", state.eePose.orientation.w,
//           state.eePose.orientation.x, state.eePose.orientation.y, state.eePose.orientation.z);
//
//    std::cout << "EE Twist --------------" << std::endl;
//    printf("EE twist: % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f\n", state.eeTwist.linear.x, state.eeTwist.linear.y,
//           state.eeTwist.linear.z, state.eeTwist.angular.x, state.eeTwist.angular.y, state.eeTwist.angular.z);
//
//    std::cout << "EE Wrench --------------" << std::endl;
//    printf("EE wrench: % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f\n", state.eeWrench.linear.x,
//           state.eeWrench.linear.y, state.eeWrench.linear.z, state.eeWrench.angular.x, state.eeWrench.angular.y,
//           state.eeWrench.angular.z);
//
//    const char map[3] = {'X', 'Y', 'Z'};
//    for (std::size_t dof = 0; dof < 6; ++dof) {
//      std::string space = dof < 3 ? "linear " : "angular";
//      printf("Jacobian %s %c: ", space.c_str(), map[dof % 3]);
//      for (std::size_t joint = 0; joint < 6; ++joint) {
//        printf("% 5.2f, ", state.jacobian[dof + joint * 6]);
//      }
//      printf("% 5.2f\n", state.jacobian[dof + 6 * 6]);
//    }
//
//    for (std::size_t row = 0; row < 7; ++row) {
//      printf("Inertial: ");
//      for (std::size_t column = 0; column < 6; ++column) {
//        printf("% 5.2f, ", state.mass[row + column * 7]);
//      }
//      printf("% 5.2f\n", state.mass[row + 6 * 7]);
//    }
//    count = 0;
//  }
//  ++count;
//}
//
//int main(int argc, char** argv) {
//
//  // Set up ZMQ
//  zmq::context_t context;
//  zmq::socket_t publisher, subscriber;
//  network::configure(context, publisher, subscriber);
//
//  frankalwi::proto::StateMessage<7> state{};
//  frankalwi::proto::CommandMessage<7> command{};
//
//  auto start = std::chrono::system_clock::now();
//  int iterations;
//  bool received = false;
//  while (subscriber.connected()) {
//    std::cout << "hello" << std::endl;
//    // blocking receive until we get a state from the robot
//    if (frankalwi::proto::receive(subscriber, state)) {
//      if (!received) {
//        received = true;
//        start = std::chrono::system_clock::now();
//        iterations = 0;
//      }
//      std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
//      throttledPrintState(state, 500, iterations / elapsed_seconds.count());
//
//      // compute the desired command here
//      command.jointTorque[0] = 0.01;
//      command.jointTorque[1] = 0.02;
//      command.jointTorque[2] = 0.03;
//      command.jointTorque[3] = 0.04;
//      command.jointTorque[4] = 0.05;
//      command.jointTorque[5] = 0.06;
//      command.jointTorque[6] = 0.07;
//      frankalwi::proto::send(publisher, command);
//      ++iterations;
//    }
//  }
//}
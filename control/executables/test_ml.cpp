#include <chrono>
#include <fstream>
#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/null_function.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>

#include <limbo/serialize/text_archive.hpp>

using namespace limbo;

struct Params {
  struct kernel_exp {
    BO_PARAM(double, sigma_sq, 0.7361);
    BO_PARAM(double, l, 0.1352);
  };
  struct kernel {
    BO_PARAM(double, noise, 0.7071);
    BO_PARAM(bool, optimize_noise, false);
  };
};

int main(int argc, char** argv) {
  // Create an input filestream
  std::vector<std::pair<std::string, std::vector<Eigen::VectorXd>>> result;
  std::ifstream myFile("/tmp/train.csv");
  if (!myFile.is_open()) { throw std::runtime_error("Could not open file"); }
  std::string line, colname;
  double val;
  Eigen::VectorXd valEigen = Eigen::Matrix<double, 1, 1>::Zero();
  if (myFile.good()) {
    std::getline(myFile, line);
    std::stringstream ss(line);
    while (std::getline(ss, colname, ',')) {
      result.push_back({colname, std::vector<Eigen::VectorXd>{}});
    }
  }
  // Read data, line by line
  while (std::getline(myFile, line)) {
    std::stringstream ss(line);
    int colIdx = 0;
    while (ss >> val) {
      valEigen.x() = val;
      result.at(colIdx).second.push_back(valEigen);
      if (ss.peek() == ',') { ss.ignore(); }
      colIdx++;
    }
  }
  myFile.close();
//  for (auto i = result.at(0).second.begin(); i != result.at(0).second.end(); ++i) {
//    std::cout << i->x() << ' ';
//  }

  std::vector<Eigen::VectorXd> samples = result.at(0).second;
  std::vector<Eigen::VectorXd> observations = result.at(1).second;
//  std::vector<double> observations;
//  for (auto i = result.at(0).second.begin(); i != result.at(0).second.end(); ++i) {
//    observations.push_back(i->x());
//  }

  // the type of the GP
  using Kernel_t = kernel::Exp<Params>;
  using Mean_t = mean::NullFunction<Params>;
  using GP_t = model::GP<Params, Kernel_t, Mean_t>;

  // 1-D inputs, 1-D outputs
  GP_t gp(1, 1);
  std::cout << gp.kernel_function().noise() << std::endl;

  // compute the GP
  gp.compute(samples, observations);
  gp.save<serialize::TextArchive>("/tmp/myGP");

//  std::vector<Eigen::VectorXd> execution;
//  double delta = (samples.back().x() - samples.front().x()) / 1000;
//  std::cout << delta << std::endl;
//  for (std::size_t i = 0; i < 1000; ++i) {
//    Eigen::VectorXd x(1);
//    x.x() = samples.front().x() + i * delta;
//    execution.emplace_back(x);
//  }
//
//  Eigen::VectorXd mu;
//  double sigma;
//  auto start = std::chrono::system_clock::now();
//  for (const auto& x : execution) {
//    std::tie(mu, sigma) = gp.query(x);
//  }
//  auto elapsed_seconds = std::chrono::system_clock::now() - start;
//  std::cout << elapsed_seconds.count() / 1e9 << std::endl;
//  std::cout << elapsed_seconds.count() / 1e12 << std::endl;

  auto start = std::chrono::system_clock::now();
  std::ofstream outputFile;
  outputFile.open("/tmp/myGP/test.csv", std::ofstream::out | std::ofstream::trunc);
  for (const auto& sample : gp.samples()) {
    Eigen::VectorXd mu;
    double sigma;
    std::tie(mu, sigma) = gp.query(sample);
    outputFile << mu << ", " << sigma << std::endl;
  }
  outputFile.close();
  auto elapsed_seconds = std::chrono::system_clock::now() - start;
  std::cout << elapsed_seconds.count() << std::endl;
  std::cout << elapsed_seconds.count() / gp.samples().size() << std::endl;
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
//  using Kernel2_t = kernel::SquaredExpARD<Params>;
//  using Mean_t = mean::Data<Params>;
//  using GP2_t = model::GP<Params, Kernel2_t, Mean_t, model::gp::KernelLFOpt<Params>>;
//
//  GP2_t gp_ard(1, 1);
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
//
//  // Sometimes is useful to save an optimized GP
//  gp_ard.save<serialize::TextArchive>("myGP");
//
//  // Later we can load -- we need to make sure that the type is identical to the one saved
//  gp_ard.load<serialize::TextArchive>("myGP");
  return 0;
}
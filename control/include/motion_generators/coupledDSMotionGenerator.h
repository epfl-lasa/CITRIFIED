//
// Created by Rui Wu on 2021/4/16.
//

#ifndef __COUPLEDDS_MOTION_GENERATOR_H__
#define __COUPLEDDS_MOTION_GENERATOR_H__


#include <signal.h>
#include <pthread.h>
#include "Eigen/Eigen"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <mutex>
#include <array>

#include "mathlib/MathLib.h"
#include "fast_gmm/GMRDynamics.h"
#include "fast_gmm/CDDynamics.h"

// /// file for online-ESN
// #include "ESN.h"
// #include "filters.h"
// #include "TimeWindow.h"
//#include "WRfilter.h"
// /////

// //  using online filter
// #include "Iir.h"

//#include <dynamic_reconfigure/server.h>
//#include <ds_motion_generator/seDS_paramsConfig.h>

#include <sstream>		
#include <iostream>		
#include <fstream>		
#include <string>		
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <boost/bind.hpp>
#include <math.h>
#include <fast_gmm/GMRDynamics.h>
#include <fast_gmm/CDDynamics.h>

#include "state_representation/space/cartesian/CartesianState.hpp"

using namespace state_representation;

//namespace motion_generator {

class coupledDSMotionGenerator {

private:
//----- SEDS
    int K_gmm_;
    int dim_;
    double dt_;
    double max_desired_vel_;
    double pos_error_;
    bool bPublish_DS_path_;
    double Mu_scale_, Sigma_scale_;
    std::vector<double> Priors_;
    std::vector<double> Mu_;
    std::vector<double> Sigma_;
    std::vector<double> attractor_;
    std::unique_ptr<GMRDynamics> SED_GMM_;

    static coupledDSMotionGenerator *me;
    bool _stop = false;

    // Target Orientation variables
    double qx, qy, qz, qw;

    // Filter variables
    std::unique_ptr<CDDynamics> CCDyn_filter_;
    std::unique_ptr<CDDynamics> CCDyn_filter_1_;
    std::unique_ptr<CDDynamics> CCDyn_filter_2_;

    double Wn_;
    MathLib::Vector accLimits_;
    MathLib::Vector velLimits_;
    MathLib::Vector F2mVvILimits_;
    MathLib::Vector dF2mVvILimits_;
    MathLib::Vector F2mVLimits_;
    MathLib::Vector dF2mVLimits_;

    MathLib::Vector real_pose_;
    MathLib::Vector desired_velocity_;
    MathLib::Vector Trans_pose;
    CartesianState eeInRobot_;


public:
    coupledDSMotionGenerator(
            double frequency,
            //-----SEDS
            int K_gmm,
            int dim,
            std::vector<double> Priors,
            std::vector<double> Mu,
            std::vector<double> Sigma,
            double Mu_scale,
            double Sigma_scale,
            std::vector<double> attractor
    );

    ~coupledDSMotionGenerator(void);

    bool Init();

    MathLib::Vector ComputeDesiredVelocity(const CartesianState& eeInRobot_);

private:

    bool InitializeDS();

};

//}
#endif

/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Authors: Mahdi Khoramshahi and Nadia Figueroa
 * email:   {mahdi.khoramshahi,nadia.figueroafernandez}@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the European Communitys Horizon 2020 Research and Innovation 
 * programme ICT-23-2014, grant agreement 644727-Cogimon and 643950-SecondHands.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __COUPLEDDS_MOTION_GENERATOR_H__
#define __COUPLEDDS_MOTION_GENERATOR_H__


#include <signal.h>
#include <pthread.h>
#include "Eigen/Eigen"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

//#include "ros/ros.h"
//#include "geometry_msgs/Pose.h"
//#include "geometry_msgs/Twist.h"
//#include "geometry_msgs/Quaternion.h"
//#include "geometry_msgs/PointStamped.h"
//#include "geometry_msgs/WrenchStamped.h"
//#include <std_msgs/Bool.h>
//#include <std_msgs/Float64MultiArray.h>
//#include "std_msgs/MultiArrayDimension.h"
//#include <std_msgs/Float64.h>
//#include "nav_msgs/Path.h"

#include <vector>
#include <mutex>
#include <array>

//#include "MathLib.h"
//#include "GMRDynamics.h"
//#include "CDDynamics.h"

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
//#include "Utils.h"
#include <boost/bind.hpp>
#include <math.h>

///// for GPR
//#include <limbo/kernel/exp.hpp>
//#include <limbo/kernel/matern_three_halves.hpp>
//#include <limbo/kernel/squared_exp_ard.hpp>
//#include <limbo/mean/data.hpp>
//#include <limbo/mean/null_function.hpp>
//#include <limbo/mean/constant.hpp>
//#include <limbo/model/gp.hpp>
//#include <limbo/model/gp/kernel_lf_opt.hpp>
//#include <limbo/tools.hpp>
//#include <limbo/tools/macros.hpp>
//
//#include <limbo/serialize/binary_archive.hpp>
//#include <chrono>
///////////

#include <fast_gmm/GMRDynamics.h>
#include <fast_gmm/CDDynamics.h>


//using namespace limbo;
///////  GPR model /////
//
//    struct Params {
//        struct kernel_maternthreehalves {
//            BO_PARAM(double, sigma_sq, 2.1828);
//            BO_PARAM(double, l, 0.0205);
//        };
//
//        // struct mean_constant {
//        //     ///@ingroup mean_defaults
//        //     BO_PARAM(double, constant, 0.0f);
//        // };
//        struct kernel : public defaults::kernel {
//            // BO_PARAM(bool, optimize_noise, true);
//        };
//        struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
//        };
//        struct opt_rprop : public defaults::opt_rprop {
//            // BO_PARAM(int, iterations, 20);
//            // BO_PARAM(double, eps_stop, 1e-4);
//        };
//    };
//
//	struct Params_1 {
//        struct kernel_maternthreehalves {
//            BO_PARAM(double, sigma_sq, 2.1828);
//            BO_PARAM(double, l, 0.0205);
//        };
//
//        // struct mean_constant {
//        //     ///@ingroup mean_defaults
//        //     BO_PARAM(double, constant, 0.0f);
//        // };
//        struct kernel : public defaults::kernel {
//            // BO_PARAM(bool, optimize_noise, true);
//        };
//        struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
//        };
//        struct opt_rprop : public defaults::opt_rprop {
//            // BO_PARAM(int, iterations, 20);
//            // BO_PARAM(double, eps_stop, 1e-4);
//        };
//    };
//	// the type of the GP
//		using Kernel_t = kernel::MaternThreeHalves<Params>;
//		using Mean_t = mean::NullFunction<Params>;
//		using GP_wr = model::GP<Params, Kernel_t, Mean_t>;
//
//		using Kernel_t_1 = kernel::MaternThreeHalves<Params_1>;
//		using Mean_t_1 = mean::NullFunction<Params_1>;
//		using GP_wr_1 = model::GP<Params_1, Kernel_t_1, Mean_t_1>;
//
//        // 1-D inputs, 1-D outputs
//		GP_wr gpwr1_gmr(1,1);
//		// 1-D inputs, 1-D outputs
//		GP_wr_1 gpwr1_gmr_1(1,1);
//////////////////////////////////


class coupledDSMotionGenerator {

private:


    // change with task

        // for simulator
        float rot_damp,rot_stiff,scale_imp;     
        float eig_velue[2];
        float eig_velue_init1,eig_velue_init2;
        int phase,disturbe,with_distrub,DSpath_kind,record_pose_error,record_first_i;
        int record_phase2_first_pose,wait_time,time_count,arrive_target,contact_time;
        double offsetY,offsetZ;
        double pos_err_record;
        Eigen::VectorXd real_pose_record;
        double b_control,c_control,a_control,alpha,beta,d_control;

        int phase_stop_i;
        int simulation_or_not;

////////////////////////////////

//----- SEDS
    int K_gmm_;
    int dim_;
    double dt_;
    double max_desired_vel_;
    double pos_error_;
    bool   bPublish_DS_path_;
    double Mu_scale_, Sigma_scale_;
    std::vector<double>  Priors_;
    std::vector<double>  Mu_;
    std::vector<double>  Sigma_;
    std::vector<double>  attractor_;
    std::unique_ptr<GMRDynamics> SED_GMM_;
//----- P2I
    int K_imp_gmm_;
    int dim_imp_;
    std::vector<double> Priors_imp_;
    std::vector<double> Mu_imp_;
    std::vector<double> Sigma_imp_;
    std::unique_ptr<GMRDynamics> GMR_GMM_IMP_;
//----- P2F
    int K_force_gmm_;
    int dim_force_;
    std::vector<double> Priors_force_;
    std::vector<double> Mu_force_;
    std::vector<double> Sigma_force_;
    std::vector<double> Max_like_force_;
    std::vector<double> Min_like_force_;
    std::unique_ptr<GMRDynamics> GMR_GMM_FORCE_;
    int K_force_1_gmm_;
    int dim_force_1_;
    std::vector<double> Priors_force_1_;
    std::vector<double> Mu_force_1_;
    std::vector<double> Sigma_force_1_;
    std::vector<double> Max_like_force_1_;
    std::vector<double> Min_like_force_1_;
    std::unique_ptr<GMRDynamics> GMR_GMM_FORCE_1_;
    int K_force_2_gmm_;
    int dim_force_2_;
    std::vector<double> Priors_force_2_;
    std::vector<double> Mu_force_2_;
    std::vector<double> Sigma_force_2_;
    std::vector<double> Max_like_force_2_;
    std::vector<double> Min_like_force_2_;
    std::unique_ptr<GMRDynamics> GMR_GMM_FORCE_2_;
    int K_force_3_gmm_;
    int dim_force_3_;
    std::vector<double> Priors_force_3_;
    std::vector<double> Mu_force_3_;
    std::vector<double> Sigma_force_3_;
    std::vector<double> Max_like_force_3_;
    std::vector<double> Min_like_force_3_;
    std::unique_ptr<GMRDynamics> GMR_GMM_FORCE_3_;
//----- F2vI
    int K_F2mVvI_gmm_;
    int dim_F2mVvI_;
    std::vector<double> Priors_F2mVvI_;
    std::vector<double> Mu_F2mVvI_;
    std::vector<double> Sigma_F2mVvI_;
    std::unique_ptr<GMRDynamics> GMR_GMM_F2mVvI_;
    //----- F2mV
    int K_F2mV_gmm_;
    int dim_F2mV_;
    std::vector<double> Priors_F2mV_;
    std::vector<double> Mu_F2mV_;
    std::vector<double> Sigma_F2mV_;
    std::unique_ptr<GMRDynamics> GMR_GMM_F2mV_;



    static coupledDSMotionGenerator* me;
    bool _stop = false;

    // Target Orientation variables
    double                qx, qy, qz, qw;

	// Filter variables
	std::unique_ptr<CDDynamics> CCDyn_filter_;
    std::unique_ptr<CDDynamics> CCDyn_filter_1_;
    std::unique_ptr<CDDynamics> CCDyn_filter_2_;

	double Wn_;
	Eigen::VectorXd accLimits_;
	Eigen::VectorXd velLimits_;
    Eigen::VectorXd F2mVvILimits_;
    Eigen::VectorXd dF2mVvILimits_;
    Eigen::VectorXd F2mVLimits_;
    Eigen::VectorXd dF2mVLimits_;


//    // ROS system variables
//    ros::NodeHandle           nh_;
//    ros::Rate                 loop_rate_;
//
//    // Publishers/Subscriber
//    ros::Subscriber           sub_real_pose_;
//    ros::Subscriber           sub_real_vel_;
//    ros::Subscriber           sub_desired_target_;
//    ros::Subscriber           sub_force_;
//    ros::Publisher            pub_desired_twist_;
//    ros::Publisher            pub_desire_imp_;
//    ros::Publisher            pub_desire_force_;
//    ros::Publisher            pub_vel_error_;
//    ros::Publisher            pub_force_error_;
//    ros::Publisher            pub_desired_orientation_;
//    ros::Publisher            pub_desired_twist_filtered_;
//    ros::Publisher            pub_target_;
//    ros::Publisher            pub_DesiredPath_;
//    ros::Publisher            pub_tigger_passive_ds_;
//    ros::Publisher            pub_desired_ori_;
//    ros::Publisher            pub_desired_passive_ds_eig_;
//    ros::Publisher            pub_desired_passive_ds_damping_;
//    ros::Publisher            pub_desired_passive_ds_stiffness_;

    // Topic Names
    std::string               input_topic_name_;
    std::string               input_real_vel_name_;
    std::string               input_target_topic_name_;
    std::string               input_force_name_;
    std::string               output_topic_name_;
    std::string               output_filtered_topic_name_;
    std::string               output_ori_topic_name_;
    std::string               output_passive_ds_eig_topic_name_;
    std::string               output_passive_ds_damping_topic_name_;
    std::string               output_passive_ds_stiffness_topic_name_;


    // Messages
//    std_msgs::Bool                  msg_passive_ds_;
//    geometry_msgs::Pose             msg_real_pose_;
//    geometry_msgs::Twist             msg_real_vel_;
//    geometry_msgs::Pose             msg_desired_target_;
//    geometry_msgs::WrenchStamped    msg_force_;
//    geometry_msgs::Quaternion       msg_quaternion_;
//    geometry_msgs::Twist            msg_desired_velocity_;
//    geometry_msgs::Twist            msg_simulator_velocity_;
//    geometry_msgs::Twist            msg_gmm_imp_;
//    geometry_msgs::Twist            msg_force_err_;
//    geometry_msgs::Twist            msg_vel_err_;
//    geometry_msgs::Twist            msg_vel_after_adapt_;
//    geometry_msgs::Twist            msg_gmm_force_;
//    geometry_msgs::Twist            msg_desired_velocity_filtered_;
//    geometry_msgs::Quaternion       msg_desired_ori_;
//    std_msgs::Float64MultiArray     msg_desired_passive_ds_eig_;
//	std_msgs::Float64	            msg_desired_passive_ds_damping_;
//	std_msgs::Float64	            msg_desired_passive_ds_stiffness_;
//
//	nav_msgs::Path msg_DesiredPath_;
//	int MAX_FRAME = 70;

	//dynamic reconfig settig
//    dynamic_reconfigure::Server<ds_motion_generator::seDS_paramsConfig> dyn_rec_srv_;
//    dynamic_reconfigure::Server<ds_motion_generator::seDS_paramsConfig>::CallbackType dyn_rec_f_;


	// Class variables
	std::mutex mutex_;

	Eigen::VectorXd real_pose_;
	Eigen::VectorXd target_pose_1_;
    Eigen::VectorXd target_pose_2_;
    Eigen::VectorXd target_pose_dyn_;
	Eigen::VectorXd target_offset_;
    Eigen::VectorXd target_offset_disturb_;

    Eigen::VectorXd real_pose_ori_;
    Eigen::VectorXd target_pose_ori_;

	Eigen::VectorXd desired_velocity_;
    Eigen::VectorXd force_velocity_;
    Eigen::VectorXd desired_velocity_after_vel_adaptor;
    Eigen::VectorXd desired_velocity_filter_for_plot;
    Eigen::VectorXd velocity_with_sim_distrub_;// use to get a simulation real velocity
    Eigen::VectorXd simulator_vel_;
    Eigen::VectorXd simulator_vel_filted_;
    Eigen::VectorXd desired_velocity_prev;
	Eigen::VectorXd desired_ori_;
	Eigen::VectorXd desired_velocity_filtered_;
    Eigen::VectorXd desired_z_force_;

    Eigen::VectorXd end_force_;
    Eigen::VectorXd force_sum;
    Eigen::VectorXd end_force_zero_stable_;
    Eigen::VectorXd force_filter_output;

    Eigen::VectorXd force_after;
    Eigen::VectorXd force_now;
    Eigen::VectorXd force_before;
    Eigen::VectorXd force_d;

    Eigen::VectorXd cut_vel_error;
    Eigen::VectorXd cut_force_error;

    Eigen::VectorXd Trans_force ;

    float cut_length;
    float stop_lentgh;
    float demo_cut_length;
    Eigen::VectorXd real_vel_;Eigen::VectorXd real_vel_filter_;
    Eigen::VectorXd real_vel_ori_;

    Eigen::Matrix<double,4,1> desired_ori_com;
    std::vector<double> eig;

	double scaling_factor_;
	double ds_vel_limit_;
    double scale_desire_force;
    double adapt_vel_cut;
    double scale_f_vel;
    double scale_punch_force_vel;
    int restart;

    bool bDynamic_target_;
    double _timeInit;

    int distrub_begin_i;
    int simulation_kind;
    int distrub_kind;
    int adaptor1;
    
//    ros::Time time_start_;
    
    std::ofstream file_robot_pose_;
    std::ofstream file_robot_vel_;
    std::ofstream file_target_pose_;
    std::ofstream file_robot_param_;
    std::ofstream file_ee_force_;
    std::ofstream file_adp_vel_;
    std::ofstream file_trans_pose_;
    std::ofstream file_com_vel_;
    std::ofstream file_GMM_force_;
    std::ofstream file_GMM_imp_;
    std::ofstream file_force_err_;
    std::ofstream file_vel_err_;

    //------- set noise
	// Define random generator with Gaussian distribution
				const double mean;//
				const double stddev;//
//				std::default_random_engine generator;
//				std::normal_distribution<double> dist;

public:
	coupledDSMotionGenerator(
//	        ros::NodeHandle &n,
	                   double frequency,
                    //-----SEDS
                        int K_gmm,
                        int dim,
                        std::vector<double> Priors,
                        std::vector<double> Mu,
                        std::vector<double> Sigma,
                        double Mu_scale,
                        double Sigma_scale,
                        std::vector<double> attractor,
                    //-----P2I
                        int K_imp_gmm,
                        int dim_imp,
                        std::vector<double> Priors_imp,
                        std::vector<double> Mu_imp,
                        std::vector<double> Sigma_imp,
                    //-----P2F
                        int K_force_gmm,
                        int dim_force,
                        std::vector<double> Priors_force,
                        std::vector<double> Mu_force,
                        std::vector<double> Sigma_force,
                        std::vector<double> Max_like_force,
                        std::vector<double> Min_like_force,
                        int K_force_1_gmm,
                        int dim_force_1,
                        std::vector<double> Priors_force_1,
                        std::vector<double> Mu_force_1,
                        std::vector<double> Sigma_force_1,
                        std::vector<double> Max_like_force_1,
                        std::vector<double> Min_like_force_1,
                        int K_force_2_gmm,
                        int dim_force_2,
                        std::vector<double> Priors_force_2,
                        std::vector<double> Mu_force_2,
                        std::vector<double> Sigma_force_2,
                        std::vector<double> Max_like_force_2,
                        std::vector<double> Min_like_force_2,
                        int K_force_3_gmm,
                        int dim_force_3,
                        std::vector<double> Priors_force_3,
                        std::vector<double> Mu_force_3,
                        std::vector<double> Sigma_force_3,
                        std::vector<double> Max_like_force_3,
                        std::vector<double> Min_like_force_3,
                    //-----F2vI
                        int K_F2mVvI_gmm,
                        int dim_F2mVvI,
                        std::vector<double> Priors_F2mVvI,
                        std::vector<double> Mu_F2mVvI,
                        std::vector<double> Sigma_F2mVvI,
                        //-----F2mV
                        int K_F2mV_gmm,
                        int dim_F2mV,
                        std::vector<double> Priors_F2mV,
                        std::vector<double> Mu_F2mV,
                        std::vector<double> Sigma_F2mV,
                    //------ROS
                        std::string input_topic_name,
                        std::string input_real_vel_name,
                        std::string output_topic_name,
                        std::string output_filtered_topic_name,
                        std::string output_ori_topic_name,
                        std::string output_passive_ds_eig_topic_name,
                        std::string output_passive_ds_damping_topic_name,
                        std::string output_passive_ds_stiffness_topic_name,
                        std::string input_target_topic_name,
                        std::string input_force_name,
                        bool bPublish_DS_path,
                        bool bDynamic_target);

    ~coupledDSMotionGenerator(void);

	bool Init();

	void Run();

    void testGPR();

private:

	bool InitializeDS();

    bool InitializeESN();

	bool InitializeROS();

    static void stopNode(int sig);

//	void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg_pos);
//
//    void UpdateRealVel(const geometry_msgs::Twist::ConstPtr& msg_vel);
//
//    void UpdateDynamicTarget(const geometry_msgs::Pose::ConstPtr& msg_tag);
//
//    void UpdateForce(const geometry_msgs::WrenchStamped& msg_force);

    void ComputeAllError();

    void filterloop();

    void forceDerivite();
    
	void ComputeDesiredVelocity();

	void PublishDesiredVelocity();

    void PublishDesiredOrient();

    void ComputeDesiredOrientation();

    void PublishDesiredOrientation();

	void PublishFuturePath();

//    void DynCallback(ds_motion_generator::seDS_paramsConfig &config, uint32_t level);

    Eigen::Matrix<double,4,1> slerp(Eigen::VectorXd starting, Eigen::VectorXd ending, double pos_error_ );

    Eigen::Matrix<double,3,3> quaternionToRotationMatrix(Eigen::Matrix<double,4,1> q);

};


#endif

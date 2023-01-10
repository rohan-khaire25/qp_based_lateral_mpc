#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <Eigen/Sparse>

#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/VehicleStatus.h>
#include <geometry_msgs/TwistStamped.h>
#include "mpc_qp_control/mpc_utils.h"
#include "mpc_qp_control/mpc_trajectory.h"
#include "mpc_qp_control/lowpass_filter.h"

#include <qp_solver_collection/QpSolverCollection.h>

class MPCQPControl
{
  public:
    MPCQPControl();

  private:
    ros::NodeHandle nh_;                    //!< @brief ros node handle
    ros::Publisher pub_steer_vel_ctrl_cmd_; //!< @brief topic publisher for control command
    ros::Publisher pub_twist_cmd_;          //!< @brief topic publisher for twist command
    ros::Subscriber sub_ref_path_;          //!< @brief topic subscriber for reference waypoints
    ros::Subscriber sub_data_;
    ros::Timer timer_control_;              //!< @brief timer for control command computation
    autoware_msgs::Lane current_waypoints_;
    
    MPCTrajectory ref_traj_;                 //!< @brief reference trajectory to be followed 

    // Variables
    int horizon_length_ = 70;
    double wheelbase_ = 3.22;
    double steering_lim_deg_ = 35.0;
    double steer_lim_rad_ = 0.61;
    double DT = 0.1;
    double vel_cmd;
    double acc_cmd;
    double ctrl_period_ = 0.02;
    double traj_resample_dist_ = 0.13;
    bool enable_path_smoothing_ = true;
    bool enable_yaw_recalculation_ = true;
    int curvature_smoothing_num_ = 30;
    int path_smoothing_times_ = 1;
    int path_filter_moving_ave_num_ = 15;

    //std::vector<double> mpc_time_v;
    //MPCTrajectory mpc_resampled_ref_traj;
    // Using Kinematic model
    int DIM_X = 2;
    int DIM_U = 1;
    int DIM_Y = 2;
  
    bool data_recieved_ = false;
    bool sparse_formulation_ = false;
    double err_lat_;
    double err_yaw_;
    double curr_vel_;
    double curr_steer_;
    double curr_curvature_;
    double mpc_curr_time;
    double u_sat_;
    
    // Functions
    void computeStateDynamics(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, double &wb_, const double &curvature_, 
                                          const double &velocity_, double &steer_lim_, int &dim_x_);
    void computeHessianAndGradient(Eigen::MatrixXd &Aex, Eigen::MatrixXd &Bex, Eigen::MatrixXd &Cex, Eigen::MatrixXd &Wex,
                                             Eigen::MatrixXd &Qex, Eigen::MatrixXd &Rex, Eigen::VectorXd &x0, Eigen::MatrixXd &Urefex, Eigen::MatrixXd &H, 
                                             Eigen::VectorXd &f);

    // Timer
    void timerCallback(const ros::TimerEvent &);

    // Callbacks
    void callbackRefPath(const autoware_msgs::Lane::ConstPtr &);
    void callbackData(const std_msgs::Float64MultiArray::ConstPtr &);
    void publishTwist(const double &vel_cmd, const double &omega_cmd);
    void publishCtrlCmd(const double &vel_cmd, const double &acc_cmd, const double &steer_cmd);
};
#include <mpc_qp_control/mpc_qp_control.h>

MPCQPControl::MPCQPControl()
{
  timer_control_ = nh_.createTimer(ros::Duration(ctrl_period_), &MPCQPControl::timerCallback, this);
  sub_ref_path_ = nh_.subscribe("/mpc_waypoints", 1, &MPCQPControl::callbackRefPath, this);
  sub_data_ = nh_.subscribe("/mpc_follower/debug/debug_values", 1, &MPCQPControl::callbackData, this);
  pub_twist_cmd_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_raw", 1);
  pub_steer_vel_ctrl_cmd_ = nh_.advertise<autoware_msgs::ControlCommandStamped>("ctrl_raw", 1);
}

void MPCQPControl::timerCallback(const ros::TimerEvent &te)
{
  if (data_recieved_ == false || ref_traj_.size() == 0)
  {
    ROS_WARN("Data not recieved");
    return;
  }

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(DIM_X);
  x0(0) = err_lat_;
  x0(1) = err_yaw_;

  // Initializing required matrices for computing QP matrices
  Eigen::MatrixXd A_k(DIM_X, DIM_X);
  Eigen::MatrixXd B_k(DIM_X, DIM_U);
  Eigen::MatrixXd W_k(DIM_X, 1);
  Eigen::MatrixXd C_k(DIM_Y, DIM_X);
  Eigen::MatrixXd Aex = Eigen::MatrixXd::Zero(DIM_X * horizon_length_, DIM_X);
  Eigen::MatrixXd Bex = Eigen::MatrixXd::Zero(DIM_X * horizon_length_, DIM_U * horizon_length_);
  Eigen::MatrixXd Wex = Eigen::MatrixXd::Zero(DIM_X * horizon_length_, 1);
  Eigen::MatrixXd Cex = Eigen::MatrixXd::Zero(DIM_Y * horizon_length_, DIM_X * horizon_length_);
  Eigen::MatrixXd Qex = Eigen::MatrixXd::Zero(DIM_Y * horizon_length_, DIM_Y * horizon_length_);
  Eigen::MatrixXd Rex = Eigen::MatrixXd::Zero(DIM_U * horizon_length_, DIM_U * horizon_length_);
  Eigen::MatrixXd Urefex = Eigen::MatrixXd::Zero(DIM_U * horizon_length_, 1);
  Eigen::MatrixXd Uref(DIM_U, 1);
  
  // Initialize the hessian matrix
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(DIM_U * horizon_length_, DIM_U * horizon_length_);
  Eigen::SparseMatrix<double> Hs(DIM_X* horizon_length_+DIM_U*(horizon_length_-1), DIM_X*horizon_length_+DIM_U*(horizon_length_-1));

  // Initialize gradient vector
  Eigen::VectorXd f;

  // Initialize constraint matrix for sparse formulation
  //Eigen::SparseMatrix<double> complete_sparse_mat(2*(DIM_X * horizon_length_) + DIM_U * horizon_length_,DIM_X * horizon_length_ + DIM_U * horizon_length_);
  Eigen::SparseMatrix<double> eq_mat(DIM_X * horizon_length_, (DIM_X * horizon_length_ + DIM_U * (horizon_length_-1)));
  //Eigen::SparseMatrix<double> ineq_mat((DIM_X * horizon_length_) + DIM_U * (horizon_length_-1), (DIM_X * horizon_length_) + DIM_U * (horizon_length_-1));
  Eigen::VectorXd eq_vec = Eigen::VectorXd::Zero(DIM_X * horizon_length_);
  //Eigen::VectorXd ineq_vec = Eigen::VectorXd::Zero(DIM_X * horizon_length_ + DIM_U * (horizon_length_-1));
  // Initialize constraint vectors - max & min
  Eigen::VectorXd ub;
  Eigen::VectorXd lb;

  // Initialize inequality constraints
  Eigen::MatrixXd A;
  Eigen::MatrixXd ubA;
  Eigen::MatrixXd lbA;

  /* weight matrix depends on the vehicle model */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DIM_U, DIM_U); 
  Eigen::MatrixXd Q_adaptive = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R_adaptive = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Q(0, 0) = 5.0;
  Q(1, 1) = 2.0;
  R(0, 0) = 1.0;

  std::vector<double> mpc_time_v;
  for (int i = 0; i < horizon_length_; ++i)
  {
    mpc_time_v.push_back(mpc_curr_time + i * DT);
  }
  MPCTrajectory mpc_resampled_ref_traj;
  if (!MPCUtils::interp1dMPCTraj(ref_traj_.relative_time, ref_traj_, mpc_time_v, mpc_resampled_ref_traj))
  {
    ROS_WARN("[MPC] calculateMPC: mpc resample error, stop mpc calculation. check code!");
  }
  // Calculate required matrices - State propagation, Hessian, gradient, limits for horizon length
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(DIM_X, DIM_X);
  if (sparse_formulation_)
  {
    // compute linear matrix constraints
    for (int i = 0; i < horizon_length_; ++i)
    {
      const double ref_k = mpc_resampled_ref_traj.k[i];
      const double ref_vx = mpc_resampled_ref_traj.vx[i];
      const double ref_vx_squared = ref_vx * ref_vx;

      /* get discrete state matrix A, B, C, W */
      MPCQPControl::computeStateDynamics(A_k, B_k, C_k, W_k, wheelbase_, ref_k, ref_vx, steer_lim_rad_, DIM_X);
      Q_adaptive = Q;
      R_adaptive = R;
      if (i == horizon_length_ - 1)
      {
        Q_adaptive(0, 0) = 2.0;
        Q_adaptive(1, 1) = 1.0;
      }
      Q_adaptive(1, 1) += ref_vx_squared * 0.5;
      R_adaptive(0, 0) += ref_vx_squared * 0.8;

      /* update mpc matrix */
      int idx_x_i = i * DIM_X;
      int idx_x_i_prev = (i-1)*DIM_X;
      int idx_u_i = i * DIM_U;
      int idx_u_i_prev = (i-1) * DIM_U;

      // Updating hessian
      Hs.insert(idx_x_i, idx_x_i) = Q_adaptive(0,0);
      Hs.insert(idx_x_i+1, idx_x_i+1) = Q_adaptive(1,1);
      if (i < (horizon_length_ - 1))
      {
        Hs.insert(i+(horizon_length_*DIM_X), i+(horizon_length_*DIM_X)) = R_adaptive(0,0);
      }
      
      // updating constraint matrix
      eq_mat.insert(idx_x_i, idx_x_i) = -1;
      eq_mat.insert(idx_x_i+1, idx_x_i+1) = -1;
      if (i > 0)
      {
        eq_mat.insert(idx_x_i,idx_x_i_prev) = A_k(0,0);
        eq_mat.insert(idx_x_i,idx_x_i_prev+1) = A_k(0,1);
        eq_mat.insert(idx_x_i+1,idx_x_i_prev) = A_k(1,0);
        eq_mat.insert(idx_x_i+1,idx_x_i_prev+1) = A_k(1,1);
      }
      if (i > 0 && i < (horizon_length_ - 1))
      {
        eq_mat.insert(idx_x_i, DIM_X*horizon_length_+idx_u_i_prev) = B_k(0,0);
        eq_mat.insert(idx_x_i+1, DIM_X*horizon_length_+idx_u_i_prev) = B_k(1,0);
      }
    }
  }
  else
  { 
    for (int i = 0; i < horizon_length_; ++i)
    {
      const double ref_k = mpc_resampled_ref_traj.k[i];
      const double ref_vx = mpc_resampled_ref_traj.vx[i];
      const double ref_vx_squared = ref_vx * ref_vx;

      /* get discrete state matrix A, B, C, W */
      MPCQPControl::computeStateDynamics(A_k, B_k, C_k, W_k, wheelbase_, ref_k, ref_vx, steer_lim_rad_, DIM_X);
      Q_adaptive = Q;
      R_adaptive = R;
      if (i == horizon_length_ - 1)
      {
        Q_adaptive(0, 0) = 2.0;
        Q_adaptive(1, 1) = 1.0;
      }
      Q_adaptive(1, 1) += ref_vx_squared * 0.5;
      R_adaptive(0, 0) += ref_vx_squared * 0.8;

      /* update mpc matrix */
      int idx_x_i = i * DIM_X;
      int idx_x_i_prev = (i - 1) * DIM_X;
      int idx_u_i = i * DIM_U;
      int idx_y_i = i * DIM_Y;
      if (i == 0)
      {
        Aex.block(0, 0, DIM_X, DIM_X) = A_k;
        Bex.block(0, 0, DIM_X, DIM_U) = B_k;
        Wex.block(0, 0, DIM_X, 1) = W_k;
      }
      else
      {
        Aex.block(idx_x_i, 0, DIM_X, DIM_X) = A_k * Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
        for (int j = 0; j < i; ++j)
        {
          int idx_u_j = j * DIM_U;
          Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) = A_k * Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
        }
        Wex.block(idx_x_i, 0, DIM_X, 1) = A_k * Wex.block(idx_x_i_prev, 0, DIM_X, 1) + W_k;
      }
      Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = B_k;
      Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = C_k;
      Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
      Rex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

      /* get reference input (feed-forward) */
      Uref(0,0) = std::atan(wheelbase_ * ref_k);
      if (std::fabs(Uref(0, 0)) < 2.0)
      {
        Uref(0, 0) = 0.0; // ignore curvature noise
      }

      Urefex.block(i * DIM_U, 0, DIM_U, 1) = Uref;

      mpc_curr_time += DT;
    }

    // compute the constraints
    A = Eigen::MatrixXd::Zero(DIM_U * horizon_length_, DIM_U * horizon_length_);
    lbA = Eigen::MatrixXd::Zero(DIM_U * horizon_length_, 1);
    ubA = Eigen::MatrixXd::Zero(DIM_U * horizon_length_, 1);
  }

  if (sparse_formulation_)
  {
    eq_vec(0) = -x0(0);
    eq_vec(1) = -x0(1);
    lb = Eigen::VectorXd::Constant((DIM_X*horizon_length_)+DIM_U * (horizon_length_-1), -1.0);
    ub = Eigen::VectorXd::Constant((DIM_X*horizon_length_)+DIM_U * (horizon_length_-1), 1.0);
    lb.head(DIM_X*horizon_length_) *= 0.5;
    ub.head(DIM_X*horizon_length_) *= 0.5;
    lb.tail(DIM_U*(horizon_length_-1)) *= steer_lim_rad_;
    ub.tail(DIM_U*(horizon_length_-1)) *= steer_lim_rad_;

    // Initialization of ineq constraints - ineq matrix and ineq vector
    //for(int i = 0; i<(DIM_U*(horizon_length_-1)); i++)
    //{
    //  ineq_vec((DIM_X*horizon_length_)+i) = steer_lim_rad_;
    //}
    //for(int i = 0; i<(DIM_X*horizon_length_ + DIM_U*(horizon_length_-1)); i++)
    //{
    //  ineq_mat.insert(i,i) = 1;
    //}
  }
  else
  {
    // Computing Hessian and gradient and bounds on decision variables
    const Eigen::MatrixXd CB = Cex * Bex;
    const Eigen::MatrixXd QCB = Qex * CB;
    H.triangularView<Eigen::Upper>() = CB.transpose() * QCB; // NOTE: This calculation is very heavy. searching for a good way...
    H.triangularView<Eigen::Upper>() += Rex;
    H.triangularView<Eigen::Lower>() = H.transpose();
    f = (Cex * (Aex * x0 + Wex)).transpose() * QCB - Urefex.transpose() * Rex;
    lb = Eigen::VectorXd::Constant(DIM_U * horizon_length_, -steer_lim_rad_); // min steering angle
    ub = Eigen::VectorXd::Constant(DIM_U * horizon_length_, steer_lim_rad_);  // max steering angle
  }
  
  if (sparse_formulation_)
  {
    // Initialize solver variables
    int dim_var = DIM_X * horizon_length_ + DIM_U * (horizon_length_-1);
    int dim_eq = DIM_X*horizon_length_;
    //int dim_ineq = DIM_X*horizon_length_+DIM_U*(horizon_length_-1);
    int dim_ineq = 0;;
    QpSolverCollection::QpCoeff qp_coeff;
    qp_coeff.setup(dim_var, dim_eq, dim_ineq);
    qp_coeff.obj_mat_ = Hs; // 70x70
    //qp_coeff.obj_vec_ = f.transpose(); // 70x1
    qp_coeff.eq_mat_ = eq_mat;
    qp_coeff.eq_vec_ = eq_vec;
    //qp_coeff.ineq_mat_ = ineq_mat;
    //qp_coeff.ineq_vec_ = ineq_vec;
    //qp_coeff.x_min_.setConstant(-1*std::numeric_limits<double>::infinity());
    //qp_coeff.x_max_.setConstant(std::numeric_limits<double>::infinity());
    qp_coeff.x_max_ = ub;
    qp_coeff.x_min_ = lb;
    auto qp_solver = QpSolverCollection::allocateQpSolver(QpSolverCollection::QpSolverType::JRLQP);
    auto start = std::chrono::system_clock::now();
    Eigen::VectorXd u_optimal = qp_solver->solve(qp_coeff);
    double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() * 1.0e-9;  
    ROS_WARN("[MPC] calculateMPC: qp solver calculation time = %f [ms]", elapsed);
    u_sat_ = std::max(std::min(u_optimal(DIM_X*horizon_length_), steer_lim_rad_), -steer_lim_rad_);
  }
  else
  {
    // Initialize solver variables
    int dim_var = horizon_length_;
    int dim_eq = 0;
    int dim_ineq = 0;;
    QpSolverCollection::QpCoeff qp_coeff;
    qp_coeff.setup(dim_var, dim_eq, dim_ineq);
    qp_coeff.obj_mat_ = H; // 70x70
    qp_coeff.obj_vec_ = f.transpose(); // 70x1
    qp_coeff.x_max_ = ub;
    qp_coeff.x_min_ = lb;
    auto qp_solver = QpSolverCollection::allocateQpSolver(QpSolverCollection::QpSolverType::JRLQP);
    auto start = std::chrono::system_clock::now();
    Eigen::VectorXd u_optimal = qp_solver->solve(qp_coeff);
    double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() * 1.0e-9;  
    ROS_WARN("[MPC] calculateMPC: qp solver calculation time = %f [ms]", elapsed);
    /* saturation */
    u_sat_ = std::max(std::min(u_optimal(0), steer_lim_rad_), -steer_lim_rad_);
  }
  
  const double omega_cmd = curr_vel_ * std::tan(u_sat_) / wheelbase_;
  vel_cmd = ref_traj_.vx[0];
  acc_cmd = (ref_traj_.vx[1] - ref_traj_.vx[0]) / DT;

  // publish twist and control command
  MPCQPControl::publishCtrlCmd(vel_cmd, acc_cmd, u_sat_);
  MPCQPControl::publishTwist(vel_cmd, omega_cmd);
}

void MPCQPControl::callbackData(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  err_lat_ = msg->data[5];
  err_yaw_ = msg->data[8];
  curr_vel_ = std::max(msg->data[10], 0.1);
  mpc_curr_time = msg->data[17];
  data_recieved_ = true;
}

void MPCQPControl::callbackRefPath(const autoware_msgs::Lane::ConstPtr &msg)
{
  current_waypoints_ = *msg;

  MPCTrajectory traj;

  /* calculate relative time */
  std::vector<double> relative_time;
  MPCUtils::calcPathRelativeTime(current_waypoints_, relative_time);

  /* resampling */
  MPCUtils::convertWaypointsToMPCTrajWithDistanceResample(current_waypoints_, relative_time, traj_resample_dist_, traj);
  MPCUtils::convertEulerAngleToMonotonic(traj.yaw);

  /* path smoothing */
  if (enable_path_smoothing_)
  {
    for (int i = 0; i < path_smoothing_times_; ++i)
    {
      if (!MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.x) ||
          !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.y) ||
          !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.yaw) ||
          !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj.vx))
      {
        ROS_WARN("[MPC] path callback: filtering error. stop filtering");
        return;
      }
    }
  }

  /* calculate yaw angle */
  if (enable_yaw_recalculation_)
  {
    MPCUtils::calcTrajectoryYawFromXY(traj);
    MPCUtils::convertEulerAngleToMonotonic(traj.yaw);
  }

  /* calculate curvature */
  MPCUtils::calcTrajectoryCurvature(traj, curvature_smoothing_num_);
  const double max_k = *max_element(traj.k.begin(), traj.k.end());
  const double min_k = *min_element(traj.k.begin(), traj.k.end());

  /* add end point with vel=0 on traj for mpc prediction */
  const double mpc_predict_time_length = (horizon_length_ + 1) * DT + ctrl_period_;
  const double end_velocity = 0.0;
  traj.vx.back() = end_velocity; // also for end point
  traj.push_back(traj.x.back(), traj.y.back(), traj.z.back(), traj.yaw.back(),
                 end_velocity, traj.k.back(), traj.relative_time.back() + mpc_predict_time_length);

  if (!traj.size())
  {
    ROS_ERROR("[MPC] path callback: trajectory size is undesired.");
    return;
  }

  ref_traj_ = traj;
};

void MPCQPControl::computeStateDynamics(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, double &wb_, const double &curvature_, 
                                          const double &velocity_, double &steer_lim_, int &DIM_X)
{
  auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };

  /* Linearize delta around delta_r (referece delta) */
  double delta_r = atan(wb_ * curvature_);
  if (abs(delta_r) >= steer_lim_)
      delta_r = steer_lim_ * (double)sign(delta_r);
  double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));
  Ad << 0.0, velocity_,
        0.0, 0.0;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(DIM_X, DIM_X);
  Ad = I + Ad * DT;
  Bd << 0.0, velocity_ / wb_ * cos_delta_r_squared_inv;
  Bd *= DT;
  Cd << 1.0, 0.0,
        0.0, 1.0;
  Wd << 0.0,
      -velocity_ / wb_ * delta_r * cos_delta_r_squared_inv;
  Wd *= DT;
}

void MPCQPControl::computeHessianAndGradient(Eigen::MatrixXd &Aex, Eigen::MatrixXd &Bex, Eigen::MatrixXd &Cex, Eigen::MatrixXd &Wex,
                                             Eigen::MatrixXd &Qex, Eigen::MatrixXd &Rex, Eigen::VectorXd &x0, Eigen::MatrixXd &Urefex, Eigen::MatrixXd &H,
                                             Eigen::VectorXd &f) 
{
  const Eigen::MatrixXd CB = Cex * Bex; //140x70
  const Eigen::MatrixXd QCB = Qex * CB; //140x70
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB; //70x70
  H.triangularView<Eigen::Upper>() += Rex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  f = (Cex * (Aex * x0 + Wex)).transpose() * QCB - Urefex.transpose() * Rex;
}

void MPCQPControl::publishTwist(const double &vel_cmd, const double &omega_cmd)
{
  /* convert steering to twist */
  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "/base_link";
  twist.header.stamp = ros::Time::now();
  twist.twist.linear.x = vel_cmd;
  twist.twist.linear.y = 0.0;
  twist.twist.linear.z = 0.0;
  twist.twist.angular.x = 0.0;
  twist.twist.angular.y = 0.0;
  twist.twist.angular.z = omega_cmd;
  pub_twist_cmd_.publish(twist);
}

void MPCQPControl::publishCtrlCmd(const double &vel_cmd, const double &acc_cmd, const double &steer_cmd)
{
  autoware_msgs::ControlCommandStamped cmd;
  cmd.header.frame_id = "/base_link";
  cmd.header.stamp = ros::Time::now();
  cmd.cmd.linear_velocity = vel_cmd;
  cmd.cmd.linear_acceleration = acc_cmd;
  cmd.cmd.steering_angle = steer_cmd;
  pub_steer_vel_ctrl_cmd_.publish(cmd);
}
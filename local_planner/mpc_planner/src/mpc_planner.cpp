/**
 * *********************************************************
 *
 * @file: mpc_planner.cpp
 * @brief: Contains the model predicted control (MPC) local planner class
 * @author: Yang Haodong
 * @date: 2024-01-31
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <osqp/osqp.h>
#include <unsupported/Eigen/KroneckerProduct>
#include <unsupported/Eigen/MatrixFunctions>
#include <pluginlib/class_list_macros.h>

#include "mpc_planner.h"

PLUGINLIB_EXPORT_CLASS(mpc_planner::MPCPlanner, nav_core::BaseLocalPlanner)

namespace mpc_planner
{
/**
 * @brief Construct a new MPC planner object
 */
MPCPlanner::MPCPlanner() : initialized_(false), goal_reached_(false), tf_(nullptr)  //, costmap_ros_(nullptr)
{
}

/**
 * @brief Construct a new MPC planner object
 */
MPCPlanner::MPCPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : MPCPlanner()
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the MPC planner object
 */
MPCPlanner::~MPCPlanner()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void MPCPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    // base
    nh.param("goal_dist_tolerance", goal_dist_tol_, 0.2);
    nh.param("rotate_tolerance", rotate_tol_, 0.5);
    nh.param("base_frame", base_frame_, base_frame_);
    nh.param("map_frame", map_frame_, map_frame_);

    // lookahead
    nh.param("lookahead_time", lookahead_time_, 1.5);
    nh.param("min_lookahead_dist", min_lookahead_dist_, 0.3);
    nh.param("max_lookahead_dist", max_lookahead_dist_, 0.9);

    // linear velocity
    nh.param("max_v", max_v_, 0.5);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    // angular velocity
    nh.param("max_w", max_w_, 1.57);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.57);

    // iteration for ricatti solution
    nh.param("predicting_time_domain", p_, 4);
    nh.param("control_time_domain", m_, 4);

    // weight matrix for penalizing state error while tracking [x,y,theta]
    std::vector<double> diag_vec;
    Q_ = Eigen::Matrix3d::Zero();
    nh.getParam("Q_matrix_diag", diag_vec);
    for (size_t i = 0; i < diag_vec.size(); ++i)
      Q_(i, i) = diag_vec[i];

    // weight matrix for penalizing input error while tracking[v, w]
    nh.getParam("R_matrix_diag", diag_vec);
    R_ = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < diag_vec.size(); ++i)
      R_(i, i) = diag_vec[i];

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;

    target_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>("/target_point", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    ROS_INFO("MUSTOFA: MPC planner initialized!");
  }
  else
  {
    ROS_WARN("MPC planner has already been initialized.");
  }
}

/**
 * @brief Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return true if the plan was updated successfully, else false
 */
bool MPCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  ROS_INFO("SET PLAN");

  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset plan parameters
  if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_rpy_ = getEulerAngles(global_plan_.back());
    goal_reached_ = false;
  }

  return true;
}

/**
 * @brief Check if the goal pose has been achieved
 * @return true if achieved, false otherwise
 */
bool MPCPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("MPC planner has not been initialized");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}

/**
 * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
 * @param cmd_vel will be filled with the velocity command to be passed to the robot base
 * @return true if a valid trajectory was found, else false
 */
bool MPCPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("MPC planner has not been initialized");
    return false;
  }

  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);
  ROS_INFO("VELOCITY");
  // get robot position in global frame
  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, map_frame_, robot_pose_odom, robot_pose_map);
  
  // transform global plan to robot frame
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_map);

  // calculate look-ahead distance
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  double L = getLookAheadDistance(vt);

  // get the particular point on the path at the lookahead distance
  geometry_msgs::PointStamped lookahead_pt;
  double theta_trj, kappa;
  getLookAheadPoint(L, robot_pose_map, prune_plan, lookahead_pt, theta_trj, kappa);

  // current angle
  double theta = tf2::getYaw(robot_pose_map.pose.orientation);  // [-pi, pi]
  // calculate commands
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back()))
  {
    du_p_ = Eigen::Vector2d(0, 0);
    double e_theta = regularizeAngle(goal_rpy_.z() - theta);
    // orientation reached
    if (!shouldRotateToPath(std::fabs(e_theta)))
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
    }
    // orientation not reached
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_t_);
    }
  }
  else
  {
    Eigen::Vector3d s(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y, theta);  // current state
    Eigen::Vector3d s_d(lookahead_pt.point.x, lookahead_pt.point.y, theta_trj);                // desired state
    Eigen::Vector2d u_r(vt, regularizeAngle(vt * kappa));                                      // refered input
    Eigen::Vector2d u = _mpcControl(s, s_d, u_r, du_p_);
    double u_v = linearRegularization(base_odom, u[0]);
    double u_w = angularRegularization(base_odom, u[1]);
    du_p_ = Eigen::Vector2d(u_v - u_r[0], regularizeAngle(u_w - u_r[1]));
    cmd_vel.linear.x = u_v;
    cmd_vel.angular.z = u_w;
  }

  // publish lookahead pose
  target_pt_pub_.publish(lookahead_pt);

  // publish robot pose
  current_pose_pub_.publish(robot_pose_map);

  return true;
}

/**
 * @brief Execute MPC control process
 * @param s     current state
 * @param s_d   desired state
 * @param u_r   refered control
 * @param du_p  previous control error
 * @return u  control vector
 */
Eigen::Vector2d MPCPlanner::_mpcControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r,
                                        Eigen::Vector2d du_p)
{
  int dim_u = 2;
  int dim_x = 3;

  // state vector (5 x 1)
  Eigen::VectorXd x = Eigen::VectorXd(dim_x + dim_u);
  x.topLeftCorner(dim_x, 1) = s - s_d;
  x[2] = regularizeAngle(x[2]);
  x.bottomLeftCorner(dim_u, 1) = du_p;

  // original state matrix
  Eigen::Matrix3d A_o = Eigen::Matrix3d::Identity();
  A_o(0, 2) = -u_r[0] * sin(s_d[2]) * d_t_;
  A_o(1, 2) = u_r[0] * cos(s_d[2]) * d_t_;

  // original control matrix
  Eigen::MatrixXd B_o = Eigen::MatrixXd::Zero(dim_x, dim_u);
  B_o(0, 0) = cos(s_d[2]) * d_t_;
  B_o(1, 0) = sin(s_d[2]) * d_t_;
  B_o(2, 1) = d_t_;

  // state matrix (5 x 5)
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim_x + dim_u, dim_x + dim_u);
  A.topLeftCorner(dim_x, dim_x) = A_o;
  A.topRightCorner(dim_x, dim_u) = B_o;
  A.bottomLeftCorner(dim_u, dim_x) = Eigen::MatrixXd::Zero(dim_u, dim_x);
  A.bottomRightCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();

  // control matrix (5 x 2)
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dim_x + dim_u, dim_u);
  B.topLeftCorner(dim_x, dim_u) = B_o;
  B.bottomLeftCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();

  // output matrix(3 x 5)
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_x, dim_x + dim_u);
  C.topLeftCorner(dim_x, dim_x) = Eigen::Matrix3d::Identity();
  C.topRightCorner(dim_x, dim_u) = Eigen::MatrixXd::Zero(dim_x, dim_u);

  // mpc state matrix(3p x 5)
  Eigen::MatrixPower<Eigen::MatrixXd> A_pow(A);
  Eigen::MatrixXd S_x = Eigen::MatrixXd::Zero(dim_x * p_, dim_x + dim_u);
  for (int i = 0; i < p_; i++)
    S_x.middleRows(dim_x * i, dim_x) = C * A_pow(i + 1);

  // mpc control matrix(3p x 2m)
  Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(dim_x * p_, dim_u * m_);
  for (int i = 0; i < p_; i++)
  {
    for (int j = 0; j < m_; j++)
    {
      if (j <= i)
        S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = C * A_pow(i - j) * B;
      else
        S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = Eigen::MatrixXd::Zero(dim_x, dim_u);
    }
  }

  // optimization
  // min 1/2 * x.T * P * x + q.T * x
  // s.t. l <= Ax <= u
  Eigen::VectorXd Yr = Eigen::VectorXd::Zero(dim_x * p_);                              // (3p x 1)
  Eigen::MatrixXd Q = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(p_, p_), Q_);  // (3p x 3p)
  Eigen::MatrixXd R = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(m_, m_), R_);  // (2m x 2m)
  Eigen::MatrixXd P = S_u.transpose() * Q * S_u + R;                                   // (2m x 2m)
  Eigen::VectorXd q = S_u.transpose() * Q * (S_x * x - Yr);                            // (2m x 1)

  // boundary
  Eigen::Vector2d u_min(min_v_, -max_w_);
  Eigen::Vector2d u_max(max_v_, max_w_);
  Eigen::Vector2d u_k_1(du_p[0], du_p[1]);
  Eigen::Vector2d du_min(-0.2, -0.2);
  Eigen::Vector2d du_max(0.2, M_PI);
  Eigen::VectorXd U_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_min);    // (2m x 1)
  Eigen::VectorXd U_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_max);    // (2m x 1)
  Eigen::VectorXd U_r = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_r);        // (2m x 1)
  Eigen::VectorXd U_k_1 = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_k_1);    // (2m x 1)
  Eigen::VectorXd dU_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_min);  // (2m x 1)
  Eigen::VectorXd dU_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_max);  // (2m x 1)

  // constriants
  Eigen::VectorXd lower = Eigen::VectorXd::Zero(2 * dim_u * m_);  // (4m x 1)
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(2 * dim_u * m_);  // (4m x 1)
  lower.topRows(dim_u * m_) = U_min - U_k_1 - U_r;
  lower.bottomRows(dim_u * m_) = dU_min;
  upper.topRows(dim_u * m_) = U_max - U_k_1 - U_r;
  upper.bottomRows(dim_u * m_) = dU_max;

  // Calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  int ind_P = 0;
  for (int col = 0; col < dim_u * m_; ++col)
  {
    P_indptr.push_back(ind_P);
    for (int row = 0; row <= col; ++row)
    {
      P_data.push_back(P(row, col));
      // P_data.push_back(P(row, col) * 2.0);
      P_indices.push_back(row);
      ind_P++;
    }
  }
  P_indptr.push_back(ind_P);

  // Calculate affine constraints (4m x 2m)
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  int ind_A = 0;
  A_indptr.push_back(ind_A);
  for (int j = 0; j < m_; ++j)
  {
    for (int n = 0; n < dim_u; ++n)
    {
      for (int row = dim_u * j + n; row < dim_u * m_; row += dim_u)
      {
        A_data.push_back(1.0);
        A_indices.push_back(row);
        ++ind_A;
      }
      A_data.push_back(1.0);
      A_indices.push_back(dim_u * m_ + dim_u * j + n);
      ++ind_A;
      A_indptr.push_back(ind_A);
    }
  }

  // Calculate offset
  std::vector<c_float> q_data;
  for (int row = 0; row < dim_u * m_; ++row)
  {
    q_data.push_back(q(row, 0));
  }

  // Calculate constraints
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  for (int row = 0; row < 2 * dim_u * m_; row++)
  {
    lower_bounds.push_back(lower(row, 0));
    upper_bounds.push_back(upper(row, 0));
  }

  // solve
  OSQPWorkspace* work = nullptr;
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPSettings* settings = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->verbose = false;
  settings->warm_start = true;

  data->n = dim_u * m_;
  data->m = 2 * dim_u * m_;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(), P_indices.data(), P_indptr.data());
  data->q = q.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(), A_indices.data(), A_indptr.data());
  data->l = lower_bounds.data();
  data->u = upper_bounds.data();

  osqp_setup(&work, data, settings);
  osqp_solve(work);
  auto status = work->info->status_val;

  if (status < 0)
  {
    std::cout << "failed optimization status:\t" << work->info->status;
    return Eigen::Vector2d::Zero();
  }

  if (status != 1 && status != 2)
  {
    std::cout << "failed optimization status:\t" << work->info->status;
    return Eigen::Vector2d::Zero();
  }

  Eigen::Vector2d u(work->solution->x[0] + du_p[0] + u_r[0], regularizeAngle(work->solution->x[1] + du_p[1] + u_r[1]));

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return u;
}

}  // namespace mpc_planner
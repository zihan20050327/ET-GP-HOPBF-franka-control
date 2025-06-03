// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/force_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include "std_msgs/String.h"
#include <sstream>


#include <fstream>

#include <vector>
#include <string>


#include <franka/robot_state.h>

#include "std_msgs/Float64MultiArray.h"  // for torque topic


// Eigen 相关
#include <Eigen/Core>
#include <Eigen/Cholesky>  // 用于 LDLT 分解

// qpOASES 求解器（需要安装 qpOASES 库）
#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES;

#include <geometry_msgs/PoseStamped.h>  // 确保包含该头文件
#include <nav_msgs/Path.h>



namespace franka_example_controllers {

bool ForceExampleController::loadTrajectory(const std::string& file_path) {
  std::ifstream file(file_path.c_str());
  if (!file.is_open()) {
    ROS_ERROR_STREAM("ForceExampleController: Unable to open trajectory file: " << file_path);
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    if(line.empty()) continue;
    std::istringstream stream(line);
    double time;
    char comma;
    Eigen::Matrix<double, 7, 1> joint_pos;
    // 读取时间（忽略）和7个关节值
    // if (!(stream >> time >> comma 
    //       >> joint_pos(0) >> comma >> joint_pos(1) >> comma >> joint_pos(2) >> comma 
    //       >> joint_pos(3) >> comma >> joint_pos(4) >> comma >> joint_pos(5) >> comma 
    //       >> joint_pos(6))) {
    //   ROS_ERROR_STREAM("ForceExampleController: Failed to parse line: " << line);
    //   continue;
    // }
    if (!(stream
          >> joint_pos(0) >> comma >> joint_pos(1) >> comma >> joint_pos(2) >> comma 
          >> joint_pos(3) >> comma >> joint_pos(4) >> comma >> joint_pos(5) >> comma 
          >> joint_pos(6))) {
      ROS_ERROR_STREAM("ForceExampleController: Failed to parse line: " << line);
      continue;
    }

    trajectory_.push_back(joint_pos);
  }
  file.close();

  if (trajectory_.empty()) {
    ROS_ERROR("ForceExampleController: Trajectory file is empty or invalid.");
    return false;
  }
  ROS_INFO_STREAM("ForceExampleController: Loaded " << trajectory_.size() << " trajectory points.");
  return true;
}  

bool ForceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  ROS_INFO("ForceExampleController: init() function has been called!"); 


  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_desired_mass_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_desired_mass_param_node");
  dynamic_server_desired_mass_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::desired_mass_paramConfig>>(

      dynamic_reconfigure_desired_mass_param_node_);
  dynamic_server_desired_mass_param_->setCallback(
      boost::bind(&ForceExampleController::desiredMassParamCallback, this, _1, _2));


  // initial subscriber
  if (!node_handle.getParam("subscribe_topic", topic_name_)) {
        ROS_ERROR("ForceExampleController: Could not read parameter subscribe_topic");
        return false;
    }


  // 从参数服务器获取力矩话题名称 (用不到)
  std::string torque_topic;
  if (!node_handle.getParam("torque_topic", torque_topic)) {
    ROS_WARN("ForceExampleController: No torque_topic specified. Using default '/external_torque'");
    torque_topic = "/external_torque";
  }

  // 创建订阅者，用于接收外部力矩指令 (用不到)
  torque_subscriber_ = node_handle.subscribe(torque_topic, 10, &ForceExampleController::torqueCallback, this);

  // 初始化 user_torque_command_ 为 0
  user_torque_command_.setZero();

  //subscribe GP
  gp_comp_sub_ = node_handle.subscribe("/gp_compensation", 10, &ForceExampleController::gpCompensationCallback, this);

  // 在 init() 函数中：
  // ee_pose_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("end_effector_pose", 10);

  // record log// 在当前工作目录下打开（覆盖）：
  // h1_log_file_.open("h1_log.txt",            std::ios::out | std::ios::trunc);
  // xee_log_file_.open("xee_log.txt",          std::ios::out | std::ios::trunc);
  // q_log_file_.open("q_log.txt",              std::ios::out | std::ios::trunc);
  // tau_dist_log_file_.open("tau_dist.txt",    std::ios::out | std::ios::trunc);
  // gp_mu_log_file_.open("gp_mu.txt",          std::ios::out | std::ios::trunc);
  // gp_eta_log_file_.open("gp_eta.txt",        std::ios::out | std::ios::trunc);
  // h2_log_file_.open("h2_log.txt",                std::ios::out | std::ios::trunc);
  // tau_cmd_log_file_.open("tau_cmd.txt",      std::ios::out | std::ios::trunc);
  // Tx_log_file_.open("Tx.txt",                std::ios::out | std::ios::trunc);

  // // 简单检查
  // if (!q_log_file_.is_open() /* … 或其它任意一个 */) {
  //   ROS_ERROR("Can not open record file");
  //   return false;
  // }
  




  // ------------------- 新增：读取轨迹文件 ----------------------
  std::string trajectory_file;
  if (!node_handle.getParam("trajectory_file", trajectory_file)) {
    ROS_ERROR("ForceExampleController: Could not read parameter trajectory_file");
    return false;
  }
  if (!loadTrajectory(trajectory_file)) {
    return false;
  }
  trajectory_index_ = 0;

  // ------------------- 新增：初始化 PD 控制参数 ----------------------
  Kp_pd_.resize(7);
  Kd_pd_.resize(7);
  // 可根据需要从参数服务器加载 PD 参数
  Kp_pd_ << 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0;
  Kd_pd_ << 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0;
 

  debug_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("/debug_tau", 10);

 
  obstacle_center_ << 0.53, 0, 0.2;
  obstacle_radius_ = 0.03;
  hocbf_alpha1_ = 5.0;//10 see lambada // best 5
  hocbf_alpha2_ = 90.0;//70 // 100
  // 触发距离 = 5 * r
  trigger_distance_ = 5.0 * obstacle_radius_;  
  // trigger_distance_ = 5.0 * obstacle_radius_; 

  // 初始化 J_pos_prev_ 标记（用于差分估计 J̇）
  has_prev_jacobian_ = false;

  return true;

}

void ForceExampleController::messageCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO_STREAM("ForceExampleController: Received message: " << msg->data);
    
}

void ForceExampleController::torqueCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  // 用不到
  if (msg->data.size() != 7) {
    ROS_ERROR_STREAM("ForceExampleController: Received torque array of invalid size: " << msg->data.size());
    return;
  }
  // copy data to user_torque_command_ 成员变量
  for (size_t i = 0; i < 7; i++) {
    user_torque_command_[i] = msg->data[i];
  }

  // ROS_INFO_STREAM("ForceExampleController: Received external torque command: "
  //                 << user_torque_command_.transpose());
}


void ForceExampleController::gpCompensationCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
if (msg->data.size() != 8) {
  ROS_ERROR("ForceExampleController: Received gp_compensation of invalid size: %lu", msg->data.size());
  return;
}


// 7维预测+第8个值为error bound eta
for (size_t i = 0; i < 8; ++i) {
  pending_gp_data_[i] = msg->data[i];

  // ROS_INFO_STREAM("eta:" << pending_gp_data_[7] );
}
// has_pending_gp_ = true;
}



void ForceExampleController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
  tau_error_.setZero();

  // 初始化轨迹控制时的索引
  trajectory_index_ = 0;
  start_time_ = ros::Time::now();
}

void ForceExampleController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  Eigen::Matrix<double, 7, 1> tau_d, tau_cmd, tau_ext;
  Eigen::Matrix<double, 6, 1> desired_force_torque;
  

    // joint speed /velocity
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // 2) Get M, C, G
  std::array<double, 49> M_array       = model_handle_->getMass();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  // std::array<double, 7> gravity_array  = model_handle_->getGravity();

  Eigen::Map<const Eigen::Matrix<double, 7, 7>> M(M_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> C(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> G(gravity_array.data());

    // ——— 缓存 & 发 disturbance ———
    static Eigen::Matrix<double,7,1> last_r;
    static bool has_last_r = false;
    if (!has_last_r) {
      last_r.setZero();
      has_last_r = true;
    }
    Eigen::Matrix<double,7,1> tau_r = last_r;
  

    // 声明 b_dyn 和 tau_norm，使它们在整个 update() 函数中都可用
    Eigen::Matrix<double, 7, 1> b_dyn = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> tau_norm = Eigen::Matrix<double, 7, 1>::Zero();

    Eigen::Matrix<double, 7, 1> nonlinear_term;
    Eigen::Matrix<double, 7, 1> disturbance;


    static std::vector<double> multiples;
    static std::vector<bool> printed;
    static bool initFlags = false;
    static double tolerance;
    if (!initFlags) {
        for (double k = 3.0; k >= 0.4; k -= 0.2) {
            multiples.push_back(k);
            printed.push_back(false);
        }
        tolerance = 0.001;  // 根据需要设置
        initFlags = true;
    }



// 计算扰动项：τ_disturbance = 0.2*(G(q) + C(q, qdot))
  Eigen::Matrix<double, 7, 1> tau_disturbance =  0.5 * C + 0.5 * G;
  Eigen::Matrix<double, 7, 1> tau_pd;
  Eigen::Matrix<double, 7, 1> tau_nominal;

  // 3) suppose, ddq_ref=0
  Eigen::Matrix<double, 7, 1> ddq_ref;
  ddq_ref.setZero();

  // 4) 计算动力学补偿 tau_dyn = M*ddq_ref + C + G
  Eigen::Matrix<double, 7, 1> tau_dyn;
  // tau_dyn = M * ddq_ref + C + G;
  tau_dyn = M * ddq_ref + C;

  
  // 
  // Eigen::Matrix<double, 7, 1> tau_cmd = G + user_torque_command_;



  // desired_force_torque.setZero();
  // desired_force_torque(2) = desired_mass_ * -9.81;
  // tau_ext = tau_measured - gravity - tau_ext_initial_;
  // tau_d = jacobian.transpose() * desired_force_torque;
  // tau_error_ = tau_error_ + period.toSec() * (tau_d - tau_ext);
  // // FF + PI control (PI gains are initially all 0)
  // tau_cmd = tau_d + k_p_ * (tau_d - tau_ext) + k_i_ * tau_error_;

  // tau_cmd = user_torque_command_; // subscribe 

  //  
  //    
  // tau_cmd = tau_dyn + user_torque_command_;

   //********** PD 轨迹跟踪控制 **********
  if (trajectory_index_ < trajectory_.size()) {
    // 当前目标轨迹点
    Eigen::Matrix<double, 7, 1> q_des = trajectory_[trajectory_index_];
    // 打印当前轨迹索引
  
    // ROS_INFO_STREAM_THROTTLE(0.5, "Trajectory index: " << trajectory_index_);

// 2. 利用有限差分计算期望关节速度 dq_des 和加速度 ddq_des
double dt = period.toSec();
Eigen::Matrix<double, 7, 1> dq_des, ddq_des;
if (has_prev_q_des_) {
    dq_des = (q_des - prev_q_des_) / dt;
} else {
    dq_des.setZero();
}
if (has_prev_dq_des_) {
    ddq_des = (dq_des - prev_dq_des_) / dt;
} else {
    ddq_des.setZero();
}
// 更新上一时刻的期望状态
prev_q_des_ = q_des;
prev_dq_des_ = dq_des;
has_prev_q_des_ = true;
has_prev_dq_des_ = true;

// 打印当前期望加速度（q̈_des，即 qdotdotdesired）的值

// ROS_INFO_STREAM_THROTTLE(0.5, "(ddq_des): " << ddq_des.transpose());


// 3. 计算跟踪误差
dq_des.setZero();
Eigen::Matrix<double, 7, 1> e = q_des - q;         // q 为当前实际关节位置（来自 robot_state）
Eigen::Matrix<double, 7, 1> edot = dq_des - dq;      // dq 为当前实际关节速度


// 4. 根据公式计算 qddotNorm = ddq_des + Kp*(q_des - q) + Kd*(dq_des - dq)
// 其中 Kp_pd_ 与 Kd_pd_ 为你定义的 PD 增益（7维向量或对角矩阵）
Eigen::Matrix<double, 7, 1> qddotNorm = ddq_des 
    + (Kp_pd_.array() * e.array()).matrix() 
    + (Kd_pd_.array() * edot.array()).matrix();

// 5. 计算 b_dyn，按照动力学补偿通常采用 b_dyn = C + G
//  b_dyn = C + G;
b_dyn = 1 * C + 0.0 * gravity;

// 6. 计算 τₙₒᵣₘ = M * qddotNorm + b_dyn
//  tau_norm = 0.8* M * qddotNorm + 1.1 * b_dyn;
tau_norm = 1 * M * qddotNorm + b_dyn;

  // 打印 tau_norm 的值
  // ROS_INFO_STREAM_THROTTLE(0.5, "tau_norm: " << tau_norm.transpose());


    // 期望关节速度设为 0
    Eigen::Matrix<double, 7, 1> dq_des1;
    dq_des1.setZero();

    // PD 控制律：τ = Kp*(q_des - q) + Kd*(dq_des - dq)
    
    tau_pd = Kp_pd_.array() * (q_des - q).array() + Kd_pd_.array() * (dq_des - dq).array();

  Eigen::Matrix<double, 7, 1> error = q_des - q;
  Eigen::Matrix<double, 7, 1> error_dot = dq_des - dq;  // 如果 dq_des 为零，则 error_dot = -dq
  tau_nominal = M * (Kp_pd_.array() * error.array() + Kd_pd_.array() * error_dot.array()).matrix() + (C);


    // 判断是否已“到达”当前目标（各关节误差小于阈值 0.1）
    bool reached = true;
    for (size_t i = 0; i < 7; ++i) {
      if (std::fabs(q_des(i) - q(i)) > 10000.0) {
        reached = false;
        break;
      }
    }
    if (reached && trajectory_index_ < trajectory_.size() - 1) {
      trajectory_index_++;
      // ROS_INFO_STREAM("Switching to trajectory point " << trajectory_index_);
    }
    tau_cmd = tau_pd;
    // tau_cmd = tau_nominal;

  // // 最终控制命令为：τ_cmd = τ_PD + τ_disturbance + τ_compensation
  // //   gp_compensation_ 由 GP 节点订阅获得，保证其为当前补偿值
    // tau_cmd = tau_pd + tau_disturbance - gp_compensation_;
    // tau_cmd = tau_pd + tau_disturbance;

  // 在这里添加直接更新下一个轨迹点的代码：
//   if (trajectory_index_ < trajectory_.size() - 1) {
//     trajectory_index_++;
// } 
// else {
//     trajectory_index_ = trajectory_.size() - 1;
// }
} else {
// 如果轨迹结束，则保持当前位置
tau_cmd.setZero();
ROS_INFO_THROTTLE(1.0, "Trajectory finished, holding current position.");
}




  // -------------------【HOCBF】-------------------
  // 1. 获取末端位姿：利用机器人状态中齐次变换 O_T_EE（
  Eigen::Map<const Eigen::Matrix<double, 4, 4>> transform(robot_state.O_T_EE.data());
  // 提取末端位置 x (3×1)
  Eigen::Vector3d x_ee = transform.block<3,1>(0,3);

  // 2. 提取位置雅可比：取 jacobian 的前 3 行 (3×7)
  Eigen::Matrix<double, 3, 7> J_pos = jacobian.topRows(3);

  // 3. 用向后差分法估计 J̇（位置部分）
  Eigen::Matrix<double, 3, 7> Jdot;
  if (has_prev_jacobian_) {
    // Jdot = (J_pos - J_pos_prev_) / period.toSec();// 0.001
    Jdot = (J_pos - J_pos_prev_) / period.toSec();// 0.001
  } else {
    Jdot.setZero();
    has_prev_jacobian_ = true;
  }
  // 更新历史雅可比（用于下一次差分计算）
  J_pos_prev_ = J_pos;

  
 

  // 4. 计算 HOCBF 链式函数

  // 参数定义
double epsilon0 = 1;      // ε₀，根据需要调整
double lambda = 10.0;      // λ值，指定为100
// double eta_bar2 = 12.68;    // GP预测误差上界的平方η̄²







  // 计算安全函数 h₁ = ||x - x_obs||² - r²
double h1_original = (x_ee - obstacle_center_).squaredNorm() - obstacle_radius_ * obstacle_radius_;


// 对于h₁（对应ψ₀），选择α*₁，例如取5.0
double alpha_star1 = 5.0;
// 计算 ε(ψ₀) = ε₀ / exp(λ * h1)
double epsilon_h1 = epsilon0 * std::exp(lambda * h1_original);
// 根据论文公式16，γ₁(ψ₀, η̄²) = (ε(ψ₀) * (η̄²)/4) / α*₁
double gamma1 = (epsilon_h1 * (eta_*eta_ / 4.0)) / alpha_star1;

double h1 = h1_original - gamma1;
// 假设 h1 已经计算：
// double h1 = (x_ee - obstacle_center_).squaredNorm() - obstacle_radius_ * obstacle_radius_;

// if (h1 < 0) {
//   static int h1_negative_count = 0;
//   ++h1_negative_count;
//   ROS_WARN_STREAM("After QP, h1 is negative (" << h1 << "). Count: " << h1_negative_count);
// } else {
//   ROS_INFO_STREAM_THROTTLE(1.0, "After QP, h1 is positive (" << h1 << ").");
// }


// 计算 h₁ 的一阶导数 h1_dot = 2*(x - x_obs)ᵀ * J_pos * dq
double h1_dot = 2.0 * (x_ee - obstacle_center_).transpose() * J_pos * dq;
double h3 = 0;

// 3. 计算 pdot1 = J_pos * dq 以及其平方范数
Eigen::Vector3d pdot1 = J_pos * dq;
double norm_pdot1_sq = pdot1.squaredNorm();

// 5. 计算 M⁻¹：使用 Eigen 的 LDLT 分解代替 .inverse()（更稳定）
Eigen::Matrix<double, 7, 7> M_inv = M.ldlt().solve(Eigen::Matrix<double, 7, 7>::Identity());
M_inv = 1 * M_inv;

 // 4. 计算 (x_ee - obstacle_center)^T * (Jdot*dq)
 double term_Jdot = (x_ee - obstacle_center_).transpose() * (Jdot * dq);


// nonlinear_term = 1 * C + 0 * G - gp_compensation_;
//  nonlinear_term = 1 * C + 0 * G - gp_compensation_;
nonlinear_term = 1 * C + 0 * G - tau_r + tau_disturbance - gp_compensation_;//GP-HOPBF
// nonlinear_term = 1.0 * C + 0 * G + tau_disturbance;//HOCBF

 // 5. 计算 (x_ee - obstacle_center)^T * J_pos * M_inv * nonlinear_term
 double term_M = (x_ee - obstacle_center_).transpose() * J_pos * M_inv * nonlinear_term;

 // 6. 计算 f_other = ||pdot1||^2 + (x_ee-obstacle_center)^T*(Jdot*dq) - (x_ee-obstacle_center)^T*J_pos*M_inv*nonlinear_term
 double f_other = norm_pdot1_sq + term_Jdot - term_M;

 // 7. 辅助变量 h2 和 h_constraint
//  double h2 = h1_dot + hocbf_alpha1_ * h1;
double h2_original = 1 * 2 * (x_ee - obstacle_center_).transpose() * pdot1  + hocbf_alpha1_ * h1;


double alpha_star2 = 30.0;  // 例如

// 计算 ε(ψ₁) = ε₀ / exp(λ * h2)
double epsilon_h2 = epsilon0 * std::exp(lambda * h2_original);

// 根据论文公式16，对于二阶系统，γ₂(ψ₁,η̄²)需要经过α*₁和α*₂的逆函数
double gamma2 = (epsilon_h2 * (eta_*eta_ / 4.0)) / (alpha_star1 * alpha_star2);

double h2 = h2_original - gamma2;








 // —— 1) 事件触发参数 & 计数器 —— 
 loop_counter_++;

 double rho           = 0.00001;    // 
 double eta_underline = 0.040; //η_underline   


//eta_(x)_max = 0.12394//
//η_underline =   
//0.000 4956times
//0.010 4975times
//0.015 4973, 4975, 4976 times
//0.020 4709, 4732, 4806, 4874 times
//0.040 1225, 1129, 1233, 1260 times
//0.060 494, 497, 519, 502times
//0.080 243, 248, 255, 225 times
//0.010 115, 106, 110, 102 times


//eta_min = 0.000 eta_max = 0.12394//

 // ε(ψ₁)
 double epsilon_phi = epsilon0 * std::exp(lambda * h2);
 // 触发阈值
 double trigger_threshold = rho * alpha_star2 * h2
                          + epsilon_phi * (eta_underline * eta_underline);
 // b 条件： ε(ψ₁) * η̄² ≥ trigger_threshold
 bool b = (epsilon_phi * (pending_gp_data_[7] * pending_gp_data_[7]) >= trigger_threshold);
 double T_x = epsilon_phi * (pending_gp_data_[7] * pending_gp_data_[7])- trigger_threshold;

 

// 2) 如果有新数据且满足条件才真正写入 gp_compensation_, eta_
// if (has_pending_gp_ && (loop_counter_ < 40.0 || (loop_counter_ >= 40.0 && b))) {
  if (loop_counter_ < 40.0 || (loop_counter_ >= 40.0 && b)) {

    // ROS_INFO_STREAM("eta:" << pending_gp_data_[7] );
   

  // 自增一次
  ++gp_update_count_;

  for (size_t i = 0; i < 7; ++i) {
    gp_compensation_(i) = pending_gp_data_[i];
  }
  eta_ = pending_gp_data_[7];
  // ROS_INFO_STREAM("GP 补偿更新 (a=" << a << ", b=" << b << ")");
}
// has_pending_gp_ = false;  // 用完就清标志





// φ₁ 基准值：
double phi_base = h1_dot + hocbf_alpha1_ * h1;

// 解析梯度：
Eigen::Vector3d dp = x_ee - obstacle_center_;        // 3×1
// 1) 2·Jdotᵀ·dp
Eigen::Matrix<double,7,1> termA = 2.0 * Jdot.transpose() * dp;
// 2) 2·J_posᵀ·(J_pos·dq)
Eigen::Matrix<double,7,1> termB = 2.0 * J_pos.transpose() * (J_pos * dq);
// 3) 2·α₁·J_posᵀ·dp
Eigen::Matrix<double,7,1> termC = 2.0 * hocbf_alpha1_ * (J_pos.transpose() * dp);

Eigen::Matrix<double,7,1> grad_phi = termA + termB + termC;

// term_qp = ‖grad_phi‖² / ε(φ₁)
double grad_norm_sq = grad_phi.squaredNorm();
double epsilon_qp  = epsilon0 * std::exp(lambda * phi_base);
double term_qp      = grad_norm_sq / epsilon_qp;

// ROS_INFO_STREAM_THROTTLE(1.0, "term_qp = " << term_qp);


//  double term_qp = 0.5;
//GP-HOPBF
 double b_constraint = f_other + ((hocbf_alpha1_ + hocbf_alpha2_) / 2.0) * h2 - (hocbf_alpha1_ * hocbf_alpha1_ / 2.0) * h1 - term_qp;
//HOCBF
//  double b_constraint = f_other + ((hocbf_alpha1_ + hocbf_alpha2_) / 2.0) * h2_original - (hocbf_alpha1_ * hocbf_alpha1_ / 2.0) * h1_original;

 double remain =  ((hocbf_alpha1_ + hocbf_alpha2_) / 2.0) * h2 - (hocbf_alpha1_ * hocbf_alpha1_ / 2.0) * h1;
 double h2_1 = 1 * 2 * (x_ee - obstacle_center_).transpose() * pdot1; 
 double h2_2 = hocbf_alpha1_ * h1;

 // 8. 计算约束矩阵 A_constraint = - (x_ee-obstacle_center)^T * J_pos * M_inv (1x7 行向量)
 Eigen::RowVectorXd A_constraint = - 1.0 * (x_ee - obstacle_center_).transpose() * J_pos * M_inv;


if ((ros::Time::now() - start_time_) >= ros::Duration(5.0) && !info_printed_) {
  double min_h1 = *std::min_element(h1_values_.begin(), h1_values_.end());
  double min_d = *std::min_element(d_values_.begin(), d_values_.end());
  // double min_h3 = *std::min_element(h3_values_.begin(), h3_values_.end());
  double min_eta_ = *std::min_element(eta_values_.begin(), eta_values_.end());
  double max_eta_ = *std::max_element(eta_values_.begin(), eta_values_.end());
  ROS_INFO_STREAM("min h1 " << min_h1 << ", h1 < 0 count: " << h1_negative_count_);
  ROS_INFO_STREAM("min d " << min_d << ", d < 0 count: " << d_negative_count_);
  // ROS_INFO_STREAM("min etax: " << min_h3 << ", h3 < 0 count: " << h3_negative_count_);
  ROS_INFO_STREAM("min eta " << min_eta_ << ", max eta " << max_eta_);
  ROS_INFO_STREAM("GP update  " << gp_update_count_ << " times");
  // ROS_INFO_STREAM("loop update  " << loop_counter_  << " times");
  // ROS_INFO_STREAM("GP update  " << etax_term  << " times");
  // ROS_INFO_STREAM("GP update  " << rho_term  << " times");
  // ROS_INFO_STREAM("GP update  " << etaunder_term << " times");
  
  info_printed_ = true;  // 设置标志，防止重复打印
}





// 如果末端距离障碍物小于触发距离，则激活 HOCBF，通过 QP 调整控制输入
if ((x_ee - obstacle_center_).norm() < trigger_distance_) {
  // ROS_INFO_STREAM_THROTTLE(0.1, "HOCBF triggered! Adjusting control input via QP.");
  
  // 调试信息
  // 在进入 QP 部分之前保存标称 tau
  // Eigen::Matrix<double, 7, 1> tau_nominal = tau_cmd;
  // ROS_INFO_STREAM("h1 = " << h1);
  // ROS_INFO_STREAM("h1_dot = " << h1_dot);
  // ROS_INFO_STREAM("f_other = " << f_other);
  // ROS_INFO_STREAM("  h2 = " << h2);
  // ROS_INFO_STREAM("  A = " << A);
  // ROS_INFO_STREAM("b_constraint = " << b_constraint);
  // ROS_INFO_STREAM("Nominal tau (PD result) = " << tau_cmd.transpose());

                   // 单独计算并打印各项 f_term 分量
  // double term1 = 2.0 * (dq.transpose() * (J_pos.transpose() * J_pos) * dq)(0,0);
  // double term2 = 2.0 * (x_ee - obstacle_center_).transpose() * (Jdot * dq);
  // double term3 = 2.0 * (x_ee - obstacle_center_).transpose() * J_pos * M_inv * (C + G);
  // ROS_INFO_STREAM("term1 = " << term1 << ", term2 = " << term2 << ", term3 = " << term3);
  

  // 构造 QP 目标：使得 τ 尽量接近 tau_cmd（PD 标称控制输入）
  // 目标函数：min ½*(τ - tau_cmd)ᵀ*(τ - tau_cmd)
  // 标准形式：min ½ τᵀ*H*τ + gᵀ*τ，其中 H = 2I, g = -2*tau_cmd
  const int n = 7;
  Eigen::Matrix<double, 7, 7> H_qp = 2.0 * Eigen::Matrix<double, 7, 7>::Identity();
  Eigen::Matrix<double, 7, 1> g_qp = -2.0 * tau_cmd;

  // QP 不等式约束为： A * τ ≥ b_constraint
  // qpOASES 要求 lbA ≤ A*τ ≤ ubA，设下界为 b_constraint，上界设为一个足够大的正数
  Eigen::Matrix<double, 1, 1> lbA;
  Eigen::Matrix<double, 1, 1> ubA;

  Eigen::MatrixXd A_qp(1, 7);
  A_qp = A_constraint;  // 1x7 矩阵
  lbA(0, 0) = -1e10;
  ubA(0, 0) = b_constraint;  // 无上界

  // 变量边界
  Eigen::Matrix<double, 7, 1> lb = -1e10 * Eigen::Matrix<double, 7, 1>::Ones();
  Eigen::Matrix<double, 7, 1> ub = 1e10 * Eigen::Matrix<double, 7, 1>::Ones();

  // 使用 qpOASES 求解 QP 问题
  QProblem qp(n, 1);
  Options options;
  options.printLevel = PL_LOW;
  qp.setOptions(options);
  int nWSR = 50;
  returnValue rval = qp.init(H_qp.data(), g_qp.data(), A_qp.data(), lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR);

  double d = (x_ee - obstacle_center_).norm();  // 当前末端距离
  // for (size_t i = 0; i < multiples.size(); i++) {
  //     double target_distance = multiples[i] * obstacle_radius_;
  //     if (!printed[i] && std::abs(d - target_distance) < tolerance) {
  //         // ROS_INFO_STREAM("At distance ~" << multiples[i] << "*r, A_constraint = " << A_constraint
  //         //                 << ", b_constraint = " << b_constraint);
  //         ROS_INFO_STREAM("At distance ~" << multiples[i] << "*r, norm_pdot1_sq = " << norm_pdot1_sq 
  //           << ", term_Jdot = " << term_Jdot 
  //           << ", term_M = " << term_M
  //           << ", f_other = " << f_other
  //           << ", remain = " << remain
  //           << ", h1 = " << h1
  //           << ", h2 = " << h2
  //           << ", h2_1 = " << h2_1
  //           << ", h2_2 = " << h2_2
  //           << ", b_constraint = " << b_constraint);
  //         printed[i] = true;
  //     }
  // }



  if (rval == SUCCESSFUL_RETURN) {
    // ROS_INFO("QP solved successfully!");
    Eigen::Matrix<double, 7, 1> tau_opt;
    qp.getPrimalSolution(tau_opt.data());
    tau_cmd = tau_opt;
    double h3 = (- A_qp * tau_cmd).value() + b_constraint;

    // 计算并打印优化后的 tau 与标称 tau 的差值
  // Eigen::Matrix<double, 7, 1> tau_diff = tau_opt - tau_nominal;
  // ROS_INFO_STREAM("Optimized tau (QP solution) = " << tau_opt.transpose());
  // ROS_INFO_STREAM("Nominal tau (PD result)   = " << tau_nominal.transpose());
  // ROS_INFO_STREAM("Difference (opt - nom)     = " << tau_diff.transpose());

  

  } else {
    ROS_WARN_STREAM("QP solve returned error code: " << rval);
    // tau_cmd = tau_nominal;
  }

  // double h3 = (A_qp * tau_cmd).value() + b_constraint;

   

  // 将 h1 和 h3 保存到对应的数组中
  h1_values_.push_back(h1_original);
  h3_values_.push_back(h3);
  d_values_.push_back(d);
  eta_values_.push_back(pending_gp_data_[7]);

  // 如果 h1 或 h3 小于 0，则计数器加 1
  if (h1 < 0) {
    ++h1_negative_count_;
  }

  if (d < 0) {
    ++d_negative_count_;
  }
  if (h3 < 0) {
    ++h3_negative_count_;
  }

//   if ((ros::Time::now() - start_time_) >= ros::Duration(5.0) && !info_printed_) {
//   double min_h1 = *std::min_element(h1_values_.begin(), h1_values_.end());
//   double min_d = *std::min_element(d_values_.begin(), d_values_.end());
//   double min_h3 = *std::min_element(h3_values_.begin(), h3_values_.end());
//   ROS_INFO_STREAM("min h1 " << min_h1 << ", h1 < 0 count: " << h1_negative_count_);
//   ROS_INFO_STREAM("min d " << min_d << ", d < 0 count: " << d_negative_count_);
//   ROS_INFO_STREAM("min h3: " << min_h3 << ", h3 < 0 count: " << h3_negative_count_);
//   ROS_INFO_STREAM("GP update  " << gp_update_count_ << " times");
//   info_printed_ = true;  // 设置标志，防止重复打印
// }


  //   // 计算 nominal 解满足约束的情况：A * tau_cmd
  //   Eigen::Matrix<double, 1, 1> constraint_value = A * tau_cmd;
  //   double diff = constraint_value(0, 0) - b_constraint;
  //   ROS_INFO_STREAM("A * tau_nom = " << constraint_value(0, 0)
  //               << ", b_constraint = " << b_constraint
  //               << ", difference = " << diff);

}

  // -------------------【HOCBF 功能集成 End】-------------------


    // 构造消息发布


    // ROS_INFO_STREAM_THROTTLE(0.5, "tau_cmd: " << tau_cmd.transpose());

  std_msgs::Float64MultiArray msg;
  msg.data.resize(7);
  for (int i = 0; i < 7; ++i) {
    msg.data[i] = tau_disturbance(i);
  }
  debug_pub_.publish(msg);

  tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }


// record log  // 假设这些变量都已经算出来了：
// Eigen::Matrix<double,7,1> gp_mu   = gp_compensation_;      
// double gp_eta                    = eta_;


// // 写入 h1 值：每行一个数字
// if (h1_log_file_.is_open()) {
//   h1_log_file_ << h1_original << "\n";
// }

// // 写入 xee 值：每行三维坐标，用空格分隔
// if (xee_log_file_.is_open()) {
//   xee_log_file_
//     << x_ee.x() << "  "
//     << x_ee.y() << "  "
//     << x_ee.z() << "\n";
// }


// // 写 q
// if (q_log_file_.is_open()) {
//   for (int i = 0; i < 7; ++i) {
//     q_log_file_ << q[i] << (i+1<7? "  " : "\n");
//   }
// }
// // 写 tau_disturbance
// if (tau_dist_log_file_.is_open()) {
//   for (int i = 0; i < 7; ++i) {
//     tau_dist_log_file_ << tau_disturbance[i] << (i+1<7? "  " : "\n");
//   }
// }
// // 写 gp_mu
// if (gp_mu_log_file_.is_open()) {
//   for (int i = 0; i < 7; ++i) {
//     gp_mu_log_file_ << gp_mu[i] << (i+1<7? "  " : "\n");
//   }
// }
// // 写 gp_eta
// if (gp_eta_log_file_.is_open()) {
//   gp_eta_log_file_ << gp_eta << "\n";
// }
// // 写 h2_original
// if (h2_log_file_.is_open()) {
//   h2_log_file_ << h2_original << "\n";
// }
// // 写 tau_cmd
// if (tau_cmd_log_file_.is_open()) {
//   for (int i = 0; i < 7; ++i) {
//     tau_cmd_log_file_ << tau_cmd[i] << (i+1<7? "  " : "\n");
//   }
// }
// // 写 T_x
// if (Tx_log_file_.is_open()) {
//   Tx_log_file_ << T_x << "\n";
// }



// ——— 更新缓存 ————
Eigen::Matrix<double,7,1> r = tau_measured - tau_cmd;
last_r = r;



  // ROS_INFO_STREAM_THROTTLE(0.1, "Current tau_cmd: " << tau_cmd.transpose());
  // ROS_INFO_STREAM_THROTTLE(0.1, "Current user_torque_command_: " << user_torque_command_.transpose());
  // ROS_INFO_STREAM_THROTTLE(0.1, "Current tau_cmd: " << tau_disturbance.transpose());
  // ROS_INFO_STREAM_THROTTLE(0.1, "Current gp_comp: " << gp_compensation_.transpose());
  // ROS_INFO_STREAM_THROTTLE(0.1, "Current tau_dis: " << tau_disturbance.transpose());

  // ROS_INFO_STREAM_THROTTLE(0.1, "Current tau_dif: " << tau_disturbance.transpose() - gp_compensation_.transpose());

  // Update signals changed online through dynamic reconfigure
  desired_mass_ = filter_gain_ * target_mass_ + (1 - filter_gain_) * desired_mass_;
  k_p_ = filter_gain_ * target_k_p_ + (1 - filter_gain_) * k_p_;
  k_i_ = filter_gain_ * target_k_i_ + (1 - filter_gain_) * k_i_;
}

void ForceExampleController::desiredMassParamCallback(
    franka_example_controllers::desired_mass_paramConfig& config,
    uint32_t /*level*/) {
  target_mass_ = config.desired_mass;
  target_k_p_ = config.k_p;
  target_k_i_ = config.k_i;
}

Eigen::Matrix<double, 7, 1> ForceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ForceExampleController,
                       controller_interface::ControllerBase)


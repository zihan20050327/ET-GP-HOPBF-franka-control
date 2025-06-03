// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>

#include <franka_example_controllers/desired_mass_paramConfig.h>

#include "std_msgs/String.h"
#include <sstream>
#include <std_msgs/Float64MultiArray.h> 

#include <nav_msgs/Path.h>

#include <fstream>


// #include <qpOASES.hpp>


namespace franka_example_controllers {

class ForceExampleController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // add  private:
  ros::Publisher debug_pub_;
  

  // // 新增：发布末端执行器位姿的 publisher
  // ros::Publisher path_pub_;
  // nav_msgs::Path path_msg_;

    // 新增：轨迹加载函数
  bool loadTrajectory(const std::string& file_path);
  ros::Subscriber subscriber_;  // 订阅者
  std::string topic_name_;      // 订阅的主题名称
  void messageCallback(const std_msgs::String::ConstPtr& msg);  // 回调函数

  private:
  ros::Subscriber torque_subscriber_;  // 用于订阅外部力矩
  Eigen::Matrix<double, 7, 1> user_torque_command_; // 存储外部接收到的力矩

  void torqueCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

  // 新增：轨迹控制相关成员变量
  std::vector<Eigen::Matrix<double,7,1>> trajectory_;
  size_t trajectory_index_;

  // 新增：PD 控制器的增益
  Eigen::Matrix<double,7,1> Kp_pd_;
  Eigen::Matrix<double,7,1> Kd_pd_;

  // 用于期望状态有限差分的变量，直接初始化
  bool has_prev_q_des_ = false;
  bool has_prev_dq_des_ = false;
  Eigen::Matrix<double, 7, 1> prev_q_des_ = Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> prev_dq_des_ = Eigen::Matrix<double, 7, 1>::Zero();

  // 用于估计 Jacobian 导数的变量
  // bool has_prev_jacobian_ = false;
  // Eigen::Matrix<double, 3, 7> J_pos_prev_ = Eigen::Matrix<double, 3, 7>::Zero();

  // HOCBF 开关，默认启用
  bool CBF_switch_ = true;

  

// 添加 GP 补偿数据订阅器与存储变量
  ros::Subscriber gp_comp_sub_;
  // Eigen::Matrix<double, 7, 1> gp_compensation_;
  // double eta_bar2_;

  int gp_update_count_{0};  // 统计 GP 更新次数

    // GP 补偿量，7 维全 0
  Eigen::Matrix<double, 7, 1> gp_compensation_{Eigen::Matrix<double, 7, 1>::Zero()};

    // 误差上界 η̄(x)²，初始设为 0
  double eta_{0.0};
  
    // 缓存新一批 GP 数据
  std::array<double, 8> pending_gp_data_{};
  bool                 has_pending_gp_{false};

   // loop计数器
   int                  loop_counter_{0};

// 声明 GP 补偿回调函数
  void gpCompensationCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);



  double desired_mass_{0.0};
  double target_mass_{0.0};
  double k_p_{0.0};
  double k_i_{0.0};
  double target_k_p_{0.0};
  double target_k_i_{0.0};
  double filter_gain_{0.001};
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  Eigen::Matrix<double, 7, 1> tau_error_;
  static constexpr double kDeltaTauMax{1.0};

// 在 ForceExampleController 类中添加

  std::vector<double> h1_values_;
  std::vector<double> h3_values_;
  std::vector<double> d_values_;
  std::vector<double> eta_values_;

  int h1_negative_count_ = 0;
  int d_negative_count_ = 0;
  int h3_negative_count_ = 0;

  ros::Time start_time_;
  bool info_printed_ = false;

  // 已有
  std::ofstream h1_log_file_;
  std::ofstream xee_log_file_;
  std::ofstream q_log_file_;               // 关节位置
  std::ofstream tau_dist_log_file_;        // 扰动
  std::ofstream gp_mu_log_file_;           // GP 均值
  std::ofstream gp_eta_log_file_;          // GP 误差上界
  std::ofstream h2_log_file_;              // h2_original
  std::ofstream tau_cmd_log_file_;        // 最终命令
  std::ofstream Tx_log_file_;              // T_x
  



  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::desired_mass_paramConfig>>
      dynamic_server_desired_mass_param_;
  ros::NodeHandle dynamic_reconfigure_desired_mass_param_node_;
  void desiredMassParamCallback(franka_example_controllers::desired_mass_paramConfig& config,
                                uint32_t level);

  // -------------------- HOCBF & Obstacle Parameters --------------------
  /**
   * @brief Center of the spherical obstacle. 
   *        Typically loaded from ROS param "obstacle_center" (size 3).
   */
  Eigen::Vector3d obstacle_center_{Eigen::Vector3d::Zero()};

  /**
   * @brief Radius of the spherical obstacle. 
   *        Typically loaded from ROS param "obstacle_radius".
   */
  double obstacle_radius_{0.0};

  /**
   * @brief Distance threshold for triggering the HOCBF-based safe control. 
   *        e.g. 5 * obstacle_radius_.
   */
  double trigger_distance_{0.0};

  /**
   * @brief HOCBF feedback parameters alpha1, alpha2.
   *        Typically loaded from ROS params "hocbf_alpha1" / "hocbf_alpha2".
   */
  double hocbf_alpha1_{5.0};
  double hocbf_alpha2_{5.0};

  // -------------------- Jacobian Differentiation --------------------
  /**
   * @brief Store the previous Jacobian (position part only, 3×7) for difference-based Jdot estimation.
   */
  Eigen::Matrix<double, 3, 7> J_pos_prev_{Eigen::Matrix<double, 3, 7>::Zero()};

  /**
   * @brief Flag indicating whether J_pos_prev_ is valid (set after first cycle).
   */
  bool has_prev_jacobian_{false};




};

}  // namespace franka_example_controllers

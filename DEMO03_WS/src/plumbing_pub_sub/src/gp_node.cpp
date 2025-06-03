// gp_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include "GPR.h"

using namespace Eigen;



class GPNode {
public:
  GPNode(ros::NodeHandle& nh) : nh_(nh) {
    // 初始化 GP 模型参数
    int x_dim = 21;  // 7位置 + 7速度 + 7期望加速度 (14维就够了，后7维目前设0)
    int y_dim = 7;   // 输出7维扰动/补偿
    int MaxDataQuantity = 50;
    double SigmaN = 0.01;
    double SigmaF = 1.0;
    VectorXd SigmaL = VectorXd::Ones(x_dim);
    VectorXd Lf_set = VectorXd::Zero(x_dim);
    double compact_range = 10.0;
    gpr_ = new GPR(x_dim, y_dim, MaxDataQuantity, SigmaN, SigmaF, SigmaL, Lf_set, compact_range);

    // 订阅 /joint_states 消息
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &GPNode::jointStateCallback, this);

    // 订阅 /debug_tau 消息 (Float64MultiArray)
    //  约定：msg.data[0..6] = tau_pd(7维)
    //        msg.data[7..13] = tau_cmd(7维)
    debug_tau_sub_ = nh_.subscribe("/debug_tau", 10, &GPNode::debugTauCallback, this);

    // 发布 GP 补偿结果到 /gp_compensation
    gp_comp_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/gp_compensation", 10);

    // 标记还没有数据
    joint_state_received_ = false;
    tau_received_ = false;
  }

  ~GPNode() {
    if (gpr_ != nullptr) {
      delete gpr_;
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber debug_tau_sub_;
  ros::Publisher gp_comp_pub_;
  GPR* gpr_;

  // 用于缓存最新的状态和力矩数据
  // joint_state
  Eigen::Matrix<double, 7, 1> current_q_;
  Eigen::Matrix<double, 7, 1> current_dq_;
  // debug_tau
  Eigen::Matrix<double, 7, 1> current_tau_pd_;
  Eigen::Matrix<double, 7, 1> current_tau_cmd_;
  Eigen::Matrix<double, 7, 1> current_tau_disturbance_;


  // 是否收到有效数据
  bool joint_state_received_;
  bool tau_received_;

private:
  // 回调1: 订阅 /joint_states
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->position.size() < 7 || msg->velocity.size() < 7) {
      ROS_ERROR("Not enough joint state data!");
      return;
    }
    // 保存前7个关节位置和速度
    for (int i = 0; i < 7; ++i) {
      current_q_(i) = msg->position[i];
      current_dq_(i) = msg->velocity[i];
    }
    joint_state_received_ = true;

    // 检查另一侧的数据是否到位
    if (tau_received_) {
      updateGP();
    }
  }

  // 回调2: 订阅 /debug_tau
  // void debugTauCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  //   // 假设 msg->data.size() == 14
  //   if (msg->data.size() < 14) {
  //     ROS_ERROR("debug_tau array size is not 14!");
  //     return;
  //   }
  //   // 前7个是 tau_pd
  //   for (int i = 0; i < 7; ++i) {
  //     current_tau_pd_(i) = msg->data[i];
  //   }
  //   // 后7个是 tau_cmd
  //   for (int i = 0; i < 7; ++i) {
  //     current_tau_cmd_(i) = msg->data[7 + i];
  //   }
  //   tau_received_ = true;

  //   // 同样检查 joint_state 是否到位
  //   if (joint_state_received_) {
  //     updateGP();
  //   }
  // }

  void debugTauCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // 假设 msg->data.size() == 14
    if (msg->data.size() < 7) {
      ROS_ERROR("debug_tau array size is not 7!");
      return;
    }
    // 前7个是 tau_pd
    for (int i = 0; i < 7; ++i) {
      current_tau_disturbance_(i) = msg->data[i];
    }
    tau_received_ = true;

    // 同样检查 joint_state 是否到位
    if (joint_state_received_) {
      updateGP();
    }
  }




  // 当 joint_states 和 debug_tau 都有新数据后，调用此函数
  void updateGP() {
    // 构造 x: 21维  [7位置 + 7速度 + 7期望加速度(先写0)]
    VectorXd x(21);
    for (int i = 0; i < 7; ++i) {
      x(i) = current_q_(i);
    }
    for (int i = 0; i < 7; ++i) {
      x(7 + i) = current_dq_(i);
    }
    // 假设期望加速度都为0（如果有更准确的值可以替换）
    for (int i = 0; i < 7; ++i) {
      x(14 + i) = 0.0;
    }

    // 计算 y：例如 y = tau_cmd - tau_pd
    //  代表“相对于 PD 的差值”
    Eigen::Matrix<double, 7, 1> y_vec = current_tau_disturbance_; 

    // 转成 VectorXd
    VectorXd y(7);
    for (int i = 0; i < 7; ++i) {
      y(i) = y_vec(i);
    }

    // 调用 GP 模型更新
    gpr_->addPoint(x, y);

    // 调用 predict
    auto predict_result = gpr_->predict(x);
    VectorXd mu = std::get<0>(predict_result);  // 7维补偿
    double eta = std::get<4>(predict_result);     // 误差界限

    // 发布到 /gp_compensation
    std_msgs::Float64MultiArray comp_msg;
    comp_msg.data.resize(8);
    for (int i = 0; i < 7; ++i) {
      comp_msg.data[i] = mu(i);
    }
    comp_msg.data[7] = eta;
    gp_comp_pub_.publish(comp_msg);

    // //打印调试信息
    // ROS_INFO_STREAM("GP update done. x=" << x.transpose() 
    //                 << ", y=" << y.transpose() 
    //                 << ", predicted mu=" << mu.transpose()
    //                 << ", error bound eta=" << eta);

    ROS_INFO_STREAM( "error bound eta=" << comp_msg.data[7]);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gp_node");
  ros::NodeHandle nh;
  GPNode gp_node(nh);
  ros::spin();
  return 0;
}

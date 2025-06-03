// #include <ros/ros.h>
// #include <sensor_msgs/JointState.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <vector>
// #include <cmath>
// #include "std_msgs/String.h"
// #include <sstream>

// // Current joint positions and velocities
// static std::vector<double> current_q(7, 0.0);    // actual joint positions
// static std::vector<double> current_dq(7, 0.0);   // actual joint velocities

// // Desired joint positions and velocities
// static std::vector<double> desired_q(7, 0.0);
// static std::vector<double> desired_dq(7, 0.0);

// // Optional integral error for PID control. If you only do PD, it's not needed.
// static std::vector<double> integral_error(7, 0.0);

// // Whether joint states have been received
// static bool got_joint_state = false;

// // PD gains
// static std::vector<double> Kp{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
// static std::vector<double> Kd{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
// static double Ki = 0.0;  // 

// // Callback Function
// void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
//   if (msg->position.size() >= 7 && msg->velocity.size() >= 7) {
//     for (int i = 0; i < 7; i++) {
//       current_q[i]  = msg->position[i];
//       current_dq[i] = msg->velocity[i];
//     }
//     got_joint_state = true;
//   }
// }

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "pd_torque_publisher_with_dt");
//   ros::NodeHandle nh;

//   // Subscribe to the joint states topic 
//   ros::Subscriber joint_sub = nh.subscribe("/joint_states", 10, jointStateCallback);
 

//   // Publish torque commands to /external_torque
//   ros::Publisher torque_pub = nh.advertise<std_msgs::Float64MultiArray>("/external_torque", 10);

//   // Set desired joint positions (example)
//   desired_q[0] = 0.0;
//   desired_q[1] = -M_PI / 4.0;
//   desired_q[2] = 0.0;
//   desired_q[3] = -3 * M_PI_4;
//   desired_q[4] = 0.0;
//   desired_q[5] =  M_PI_2;
//   // desired_q[6] =  M_PI_4;
//   desired_q[6] =  0.758154;

//   // Control loop at 200Hz
//   ros::Rate rate(1000);
//   ros::Time last_time = ros::Time::now();

//    // ------------------start time ------------------
//   ros::Time start_time = ros::Time::now();
//   // -----------------------------------------------------------



  

//   while (ros::ok()) {
//     // Compute actual time difference (dt)
//     ros::Time now = ros::Time::now();
//     double dt = (now - last_time).toSec();
//     last_time = now;

//     // Process callbacks to get the latest joint states
//     ros::spinOnce();

//     if (got_joint_state) {

//       // ------------------ compute running time ------------------
//       double elapsed = (now - start_time).toSec();
//       // 如果 elapsed < 1 秒，就直接输出零扭矩
//       bool zero_torque_mode = (elapsed < 1.0);
//       // ---------------------------------------------------------------

//       // Compute position and velocity errors
//       std::vector<double> e_q(7, 0.0);
//       std::vector<double> e_dq(7, 0.0);
//       for (int i = 0; i < 7; i++) {
//         e_q[i]  = desired_q[i]  - current_q[i];
//         e_dq[i] = desired_dq[i] - current_dq[i];
//       }

//       // If Ki is not zero, accumulate integral error
//       if (Ki > 1e-6) {
//         for (int i = 0; i < 7; i++) {
//           integral_error[i] += e_q[i] * dt;
//         }
//       }

//       // PD (or PID) control
//       // tau[i] = Kp[i]*e_q[i] + Kd[i]*e_dq[i] + Ki * integral_error[i] 
//       std_msgs::Float64MultiArray torque_msg;
//       torque_msg.layout.dim.resize(1);
//       torque_msg.layout.dim[0].size = 7;
//       torque_msg.layout.dim[0].stride = 7;
//       torque_msg.layout.data_offset = 0;
//       torque_msg.data.resize(7);

//       // 如果需要在前 3 秒发零扭矩，则直接填 0
//       if (zero_torque_mode) {
//         for (int i = 0; i < 7; i++) {
//           torque_msg.data[i] = 0.0;
//         }
//       } else {
//         // 超过 1 秒后，进行正常的 PD 计算
//         for (int i = 0; i < 7; i++) {
//           double tau_i = Kp[i] * e_q[i] + Kd[i] * e_dq[i];
//           // tau_i += Ki * integral_error[i];  // 如果要用积分，去掉注释
//           torque_msg.data[i] = tau_i;
//         }
//       }

//       // for (int i = 0; i < 7; i++) {
//       //   double tau_i = Kp[i] * e_q[i] + Kd[i] * e_dq[i];
//       //   // 
//       //   // tau_i += Ki * integral_error[i];
//       //   torque_msg.data[i] = tau_i;
//       // }

//       // Publish torque commands
//       torque_pub.publish(torque_msg);

//       // Print once per second
//        // 
//   std::ostringstream oss;
//   // oss << "current_q: [";
//   // for (size_t i = 0; i < current_q.size(); i++) {
//   //   oss << current_q[i];
//   //   if (i < current_q.size() - 1) {
//   //     oss << ", ";
//   //   }
//   // }
//   // oss << "]";

// // 利用 ROS_INFO_STREAM 打印
//       ROS_INFO_STREAM(oss.str());

//       ROS_INFO_STREAM_THROTTLE(10.0, "dt=" << dt << " | tau=["
//         << torque_msg.data[0] << ", " << torque_msg.data[1] << ", " << torque_msg.data[2] << ", "
//         << torque_msg.data[3] << ", " << torque_msg.data[4] << ", " << torque_msg.data[5] << ", "
//         << torque_msg.data[6] << "]");
//     }

//     rate.sleep();
//   }

//   return 0;
// }

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <cmath>
#include <sstream>
#include <fstream>
#include <string>

// 当前实际的关节位置和速度
static std::vector<double> current_q(7, 0.0);
static std::vector<double> current_dq(7, 0.0);

// 期望的关节位置和速度（后者默认全为0）
static std::vector<double> desired_q(7, 0.0);
static std::vector<double> desired_dq(7, 0.0);

// 如果使用 PID，可使用积分项（这里仅用 PD，可忽略）
static std::vector<double> integral_error(7, 0.0);

// 标志：是否已收到关节状态
static bool got_joint_state = false;

// PD 控制器的增益参数
static std::vector<double> Kp{15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0};
static std::vector<double> Kd{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
static double Ki = 0.0;  // 如果不使用积分，保持为0

// 订阅关节状态的回调函数
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (msg->position.size() >= 7 && msg->velocity.size() >= 7) {
    for (int i = 0; i < 7; i++) {
      current_q[i]  = msg->position[i];
      current_dq[i] = msg->velocity[i];
    }
    got_joint_state = true;
  }
}

// 从文件中读取关节轨迹，文件中每行格式应为：
// time, pos0, pos1, pos2, pos3, pos4, pos5, pos6
std::vector<std::vector<double>> readJointTrajectory(const std::string& file_path) {
  std::vector<std::vector<double>> trajectory;
  std::ifstream file(file_path);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Unable to open trajectory file: " << file_path);
    return trajectory;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream stream(line);
    double time;
    char comma;
    std::vector<double> joint_positions(7, 0.0);
    if (stream >> time >> comma >> joint_positions[0] >> comma >> joint_positions[1] >> comma >>
        joint_positions[2] >> comma >> joint_positions[3] >> comma >>
        joint_positions[4] >> comma >> joint_positions[5] >> comma >>
        joint_positions[6]) {
      trajectory.push_back(joint_positions);
    }
  }
  file.close();
  return trajectory;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pd_torque_publisher_with_dt");
  ros::NodeHandle nh;

  // 订阅关节状态话题
  ros::Subscriber joint_sub = nh.subscribe("/joint_states", 10, jointStateCallback);

  // 发布扭矩命令到 /external_torque 话题
  ros::Publisher torque_pub = nh.advertise<std_msgs::Float64MultiArray>("/external_torque", 10);

  // 读取轨迹文件（请确保文件路径正确，且文件格式符合要求）
  // std::string trajectory_file = "joint_space_trajectory_with_time.txt";
  std::string trajectory_file = "interpolated_trajectory_with_time.txt";
  
  auto trajectory = readJointTrajectory(trajectory_file);
  if (trajectory.empty()) {
    ROS_ERROR_STREAM("Trajectory file is empty or invalid!");
    return -1;
  }
  size_t trajectory_index = 0;  // 当前执行到的轨迹点索引

  // 期望速度全部为0
  desired_dq.assign(7, 0.0);

  // 控制循环频率（ 1000Hz）
  ros::Rate rate(1000);
  ros::Time last_time = ros::Time::now();
  ros::Time start_time = ros::Time::now();

  while (ros::ok()) {
    ros::Time now = ros::Time::now();
    double dt = (now - last_time).toSec();
    last_time = now;

    ros::spinOnce();

    if (got_joint_state) {
      double elapsed = (now - start_time).toSec();
      // 如果还未开始（例如前1秒内），发送零扭矩
      bool zero_torque_mode = (elapsed < 1.0);

      // 如果轨迹全部执行完，则发送零扭矩并退出节点
      if (trajectory_index >= trajectory.size()) {
        ROS_INFO_STREAM("Trajectory finished. Shutting down node.");
        std_msgs::Float64MultiArray torque_msg;
        torque_msg.data.assign(7, 0.0);
        torque_pub.publish(torque_msg);
        ros::shutdown();
        continue;
      }

      // 将当前轨迹点更新为期望的关节位置
      desired_q = trajectory[trajectory_index];

      // 计算位置和速度误差
      std::vector<double> e_q(7, 0.0);
      std::vector<double> e_dq(7, 0.0);
      for (int i = 0; i < 7; i++) {
        e_q[i]  = desired_q[i]  - current_q[i];
        e_dq[i] = desired_dq[i] - current_dq[i];
      }

      // 如使用积分环节，则累计积分误差（此处 Ki 为0，即无积分）
      if (Ki > 1e-6) {
        for (int i = 0; i < 7; i++) {
          integral_error[i] += e_q[i] * dt;
        }
      }

      std_msgs::Float64MultiArray torque_msg;
      torque_msg.layout.dim.resize(1);
      torque_msg.layout.dim[0].size = 7;
      torque_msg.layout.dim[0].stride = 7;
      torque_msg.layout.data_offset = 0;
      torque_msg.data.resize(7);

      if (zero_torque_mode) {
        for (int i = 0; i < 7; i++) {
          torque_msg.data[i] = 0.0;
        }
      } else {
        // PD 控制计算
        for (int i = 0; i < 7; i++) {
          double tau = Kp[i] * e_q[i] + Kd[i] * e_dq[i];
          if (Ki > 1e-6) {
            tau += Ki * integral_error[i];
          }
          torque_msg.data[i] = tau;
        }
      }

      // 检查是否到达当前轨迹点（所有关节误差均小于阈值，比如 0.1）
      bool reached = true;
      for (int i = 0; i < 7; i++) {
        if (std::fabs(e_q[i]) > 0.1) {
          reached = false;
          break;
        }
      }
      if (reached) {
        trajectory_index++;  // 到达当前点，转向下一个
      }

      // 发布扭矩命令
      torque_pub.publish(torque_msg);

      ROS_INFO_STREAM_THROTTLE(0.1, "dt=" << dt << " | tau=[" <<
          torque_msg.data[0] << ", " << torque_msg.data[1] << ", " << torque_msg.data[2] << ", " <<
          torque_msg.data[3] << ", " << torque_msg.data[4] << ", " << torque_msg.data[5] << ", " <<
          torque_msg.data[6] << "]");
    }
    rate.sleep();
  }
  return 0;
}

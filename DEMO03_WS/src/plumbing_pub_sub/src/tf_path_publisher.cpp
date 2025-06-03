#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_path_publisher");
  ros::NodeHandle nh;

  // 创建 tf2 的 Buffer 与 Listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // 创建一个发布器，用于发布轨迹消息（nav_msgs/Path）
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("end_effector_path", 10);

  // 初始化 nav_msgs::Path 消息，设置固定坐标系（与 TF 中的固定坐标系一致）
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "panda_link0";  // 请根据实际情况设置

  ros::Rate rate(100.0);
  while (ros::ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      // 获取最新的 TF 数据：
      // 从固定坐标系 "panda_link0" 到末端坐标系 "panda_hand"
      transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_hand_tcp", ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(0.1).sleep();
      continue;
    }

    // 构造 PoseStamped 消息保存末端执行器位置
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "panda_link0";  // 与 Path 的 frame_id 保持一致
    pose.pose.position.x = transformStamped.transform.translation.x;
    pose.pose.position.y = transformStamped.transform.translation.y;
    pose.pose.position.z = transformStamped.transform.translation.z;
    pose.pose.orientation = transformStamped.transform.rotation;  // 姿态信息

    // 将当前位姿追加到轨迹中
    path_msg.poses.push_back(pose);
    // 更新轨迹消息的时间戳
    path_msg.header.stamp = ros::Time::now();

    // 发布轨迹消息
    path_pub.publish(path_msg);

    rate.sleep();
  }

  return 0;
}

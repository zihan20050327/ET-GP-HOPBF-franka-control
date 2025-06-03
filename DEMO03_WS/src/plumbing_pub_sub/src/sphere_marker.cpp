#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sphere_marker_node");
  ros::NodeHandle nh;

  // 创建一个发布者，用于发布 visualization_msgs::Marker 类型消息
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("sphere_marker", 1);

  // 等待片刻，让 publisher 完成与 Master 的连接
  ros::Duration(0.5).sleep();

  // 构造一个 Marker 消息
  visualization_msgs::Marker marker;
  marker.header.frame_id = "panda_link0";  // 你想要可视化的坐标系
  marker.header.stamp = ros::Time::now();
  marker.ns = "my_sphere";
  marker.id = 0;

  // Marker 类型：球体
  marker.type = visualization_msgs::Marker::SPHERE;

  // 添加或更新此 Marker
  marker.action = visualization_msgs::Marker::ADD;

  // 球心的位置
  // marker.pose.position.x = 0.53;
  marker.pose.position.x = 0.53;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.2;

  // 球心的方位（四元数），这里设为单位朝向
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // 设置球体大小（直径）
  double radius = 0.03;  // 你想要的半径
  marker.scale.x = 2.0 * radius;
  marker.scale.y = 2.0 * radius;
  marker.scale.z = 2.0 * radius;

  // 设置颜色 (RGBA)
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;  // 不透明

  // 发送速度（若只想显示一次，可不循环）
  ros::Rate r(1);
  while (ros::ok()) {
    // 发布 Marker
    marker_pub.publish(marker);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

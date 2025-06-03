#include"ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"ergouzi");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("fang",10);
    
    std_msgs::String msg;

    ros::Rate rate(10);
    int count = 0;

    ros::Duration(3).sleep();
    while (ros::ok())
    {
        count++;
        // msg.data = "hello";
        std::stringstream ss;
        ss <<"hello ---"<< count;
        msg.data = ss.str();
        pub.publish(msg);
        ROS_INFO("data published is: %s ",ss.str().c_str());
        rate.sleep();
        ros::spinOnce();
    }
    

    return 0;
}

// #include <ros/ros.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <tf/transform_listener.h>

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "panda_ee_path_publisher");
//     ros::NodeHandle nh;

//     // Publisher to publish the path of panda_EE
//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("panda_ee_path", 10);

//     // TF listener to get transformations
//     tf::TransformListener listener;

//     // Path message
//     nav_msgs::Path path;
//     path.header.frame_id = "panda_hand";  // Reference frame for the path

//     ros::Rate loop_rate(10);  // Frequency of the loop in Hz

//     while (ros::ok()) {
//         try {
//             // Get the transform from panda_hand to panda_EE
//             tf::StampedTransform transform;
//             listener.lookupTransform("panda_hand", "panda_EE", ros::Time(0), transform);

//             // Create a PoseStamped message
//             geometry_msgs::PoseStamped pose;
//             pose.header.frame_id = "panda_hand";  // Reference frame for the pose
//             pose.header.stamp = ros::Time::now();

//             // Set position
//             pose.pose.position.x = transform.getOrigin().x();
//             pose.pose.position.y = transform.getOrigin().y();
//             pose.pose.position.z = transform.getOrigin().z();

//             // Set orientation
//             pose.pose.orientation.x = transform.getRotation().x();
//             pose.pose.orientation.y = transform.getRotation().y();
//             pose.pose.orientation.z = transform.getRotation().z();
//             pose.pose.orientation.w = transform.getRotation().w();

//             // Add the pose to the path
//             path.poses.push_back(pose);

//             // Limit the size of the path (to avoid memory issues)
//             if (path.poses.size() > 100) {
//                 path.poses.erase(path.poses.begin());  // Remove the oldest pose
//             }

//             // Update the header timestamp
//             path.header.stamp = ros::Time::now();

//             // Publish the path
//             path_pub.publish(path);

//         } catch (tf::TransformException &ex) {
//             ROS_WARN("Could not get transform: %s", ex.what());
//         }

//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }

// #include <ros/ros.h>
// #include <tf/transform_listener.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Path.h>

// int main(int argc, char** argv) {
//     // Initialize the ROS node
//     ros::init(argc, argv, "panda_end_effector_path");
//     ros::NodeHandle nh;

//     // Publisher for the path topic
//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/end_effector_path", 10);

//     // TF listener
//     tf::TransformListener listener;

//     // Path message
//     nav_msgs::Path path_msg;
//     path_msg.header.frame_id = "world"; // Replace with your global coordinate frame

//     ros::Rate rate(10.0); // Set loop rate to 10 Hz

//     while (ros::ok()) {
//         tf::StampedTransform transform;
//         try {
//             // Listen for the transform from the global frame to the end-effector frame
//             listener.waitForTransform("world", "panda_link8", ros::Time(0), ros::Duration(1.0));
//             listener.lookupTransform("world", "panda_link8", ros::Time(0), transform);

//             // Create a PoseStamped message
//             geometry_msgs::PoseStamped pose;
//             pose.header.frame_id = "world";
//             pose.header.stamp = ros::Time::now();

//             // Set the position
//             pose.pose.position.x = transform.getOrigin().x();
//             pose.pose.position.y = transform.getOrigin().y();
//             pose.pose.position.z = transform.getOrigin().z();

//             // Set the orientation
//             pose.pose.orientation.x = transform.getRotation().x();
//             pose.pose.orientation.y = transform.getRotation().y();
//             pose.pose.orientation.z = transform.getRotation().z();
//             pose.pose.orientation.w = transform.getRotation().w();

//             // Add the current point to the path
//             path_msg.poses.push_back(pose);

//             // Limit the number of points in the path (optional, to avoid unlimited growth)
//             if (path_msg.poses.size() > 1000) {
//                 path_msg.poses.erase(path_msg.poses.begin());
//             }

//             // Publish the path message
//             path_pub.publish(path_msg);
//         } catch (tf::TransformException& ex) {
//             ROS_WARN("%s", ex.what());
//             ros::Duration(1.0).sleep();
//             continue;
//         }

//         rate.sleep();
//     }

//     return 0;
// }








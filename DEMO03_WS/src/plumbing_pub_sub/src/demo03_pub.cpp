#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "panda_end_effector_path");
    ros::NodeHandle nh;

    // Publisher for the path topic
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/end_effector_path", 10);

    // TF listener
    tf::TransformListener listener;

    // Path message
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world"; // Replace with your global coordinate frame

    ros::Rate rate(10.0); // Set loop rate to 10 Hz

    while (ros::ok()) {
        tf::StampedTransform transform;
        try {
            // Listen for the transform from the global frame to the end-effector frame
            listener.waitForTransform("world", "panda_link8", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("world", "panda_link8", ros::Time(0), transform);

            // Create a PoseStamped message
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = ros::Time::now();

            // Set the position
            pose.pose.position.x = transform.getOrigin().x();
            pose.pose.position.y = transform.getOrigin().y();
            pose.pose.position.z = transform.getOrigin().z();

            // Set the orientation
            pose.pose.orientation.x = transform.getRotation().x();
            pose.pose.orientation.y = transform.getRotation().y();
            pose.pose.orientation.z = transform.getRotation().z();
            pose.pose.orientation.w = transform.getRotation().w();

            // Add the current point to the path
            path_msg.poses.push_back(pose);

            // Limit the number of points in the path (optional, to avoid unlimited growth)
            if (path_msg.poses.size() > 1000) {
                path_msg.poses.erase(path_msg.poses.begin());
            }

            // Publish the path message
            path_pub.publish(path_msg);
        } catch (tf::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        rate.sleep();
    }

    return 0;
}

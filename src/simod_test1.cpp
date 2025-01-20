#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "manipulator_sequence_publisher");
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("/manipulator/adm_xd",  1);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/manipulator/tcp_vel", 1);

    // Define the pose message
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = 0.10;
    pose_msg.position.y = -0.6520;
    pose_msg.position.z = 0.150;
    pose_msg.orientation.x = 0.707;
    pose_msg.orientation.y = 0.0;
    pose_msg.orientation.z = 0.0;
    pose_msg.orientation.w = 0.707;

    // Publish the pose message
    ROS_INFO("Publishing pose to /manipulator/adm_xd");
    pose_pub.publish(pose_msg);

    // Wait for 5 seconds
    ros::Duration(5.0).sleep();

    // Define the velocity message
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.1;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = 0.0;

    ROS_INFO("Publishing velocity to /manipulator/tcp_vel");
    ros::Time start_time = ros::Time::now();

    // Publish the velocity for 6 seconds
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::Time::now() - start_time < ros::Duration(6.0)) {
        vel_pub.publish(vel_msg);
        loop_rate.sleep();
    }

    // Stop the manipulator by setting velocity to 0
    vel_msg.linear.z = 0.0;
    ROS_INFO("Stopping manipulator");
    vel_pub.publish(vel_msg);

    ros::Duration(0.1).sleep(); // Give some time to ensure the stop message is published

    ROS_INFO("Sequence completed");
    return 0;
}

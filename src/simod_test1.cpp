#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "simod_test1");
    ros::NodeHandle nh;

    // Publishers
    // ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("/manipulator/desired_tcp_pose",  1);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("/manipulator/adm_xd",  1);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/manipulator/cmd_vel", 1);

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
    ROS_INFO("Wait for the other robot");
    ros::Rate loop_rate(500); // 10 Hz
    ros::Time start_time = ros::Time::now();

    while (ros::Time::now() - start_time < ros::Duration(4.0)) {
        pose_pub.publish(pose_msg);
        loop_rate.sleep();
    }

    // Wait for 5 seconds
    ros::Duration(1.0).sleep();

    // Define the velocity message
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.1;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = 0.0;

    ROS_INFO("Publishing velocity to /manipulator/cmd_vel");
    start_time = ros::Time::now();

    // Publish the velocity for 6 seconds
    while (ros::Time::now() - start_time < ros::Duration(4.0)) {
        vel_pub.publish(vel_msg);
        loop_rate.sleep();
    }

    start_time = ros::Time::now();

    // Stop the manipulator by setting velocity to 0
    vel_msg.linear.z = 0.0;
    ROS_INFO("Stopping manipulator");
    vel_pub.publish(vel_msg);

    while (ros::Time::now() - start_time < ros::Duration(2.0)) {
        vel_pub.publish(vel_msg);
        loop_rate.sleep();
    }

    start_time = ros::Time::now();

    vel_msg.linear.z = -0.10;
    ROS_INFO("Going down");
    // Publish the velocity for 6 seconds
    while (ros::Time::now() - start_time < ros::Duration(4.0)) {
        vel_pub.publish(vel_msg);
        loop_rate.sleep();
    }


    // Stop the manipulator by setting velocity to 0
    vel_msg.linear.z = 0.0;
    ROS_INFO("Stopping manipulator");
    vel_pub.publish(vel_msg);

    start_time = ros::Time::now();

    // Publish the velocity for 6 seconds
    while (ros::Time::now() - start_time < ros::Duration(1.0)) {
        vel_pub.publish(vel_msg);
        loop_rate.sleep();
    }

    ros::Duration(1).sleep(); // Give some time to ensure the stop message is published


    ROS_INFO("Sequence completed");
    return 0;
}

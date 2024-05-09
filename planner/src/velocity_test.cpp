#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// odom输出的速度数据是在机体坐标系下的

// gazebo_msgs::ModelStates gazebo_model;
geometry_msgs::Twist velocity_w;
nav_msgs::Odometry odom;

void gazeboCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    velocity_w =  msg->twist.at(2);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_test");
    ros::NodeHandle nh;
    ros::Subscriber gazebo_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, gazeboCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/odometry/in", 1, odomCallback, ros::TransportHints().tcpNoDelay());
    ros::Rate rate(60);
    while(ros::ok())
    {
        ros::spinOnce();
        Eigen::Vector3d velocity_b(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z), velocity_W_b;
        Eigen::Vector3d velocity_w_(velocity_w.linear.x, velocity_w.linear.y, velocity_w.linear.z);
        Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
        velocity_W_b = q * velocity_b;
        ROS_INFO_STREAM("the gazebo velocity and odom2wordls velocity are " << '\n' << velocity_w_.x() << " " << velocity_W_b.x() << '\n' << 
                        velocity_w_.y() << " " << velocity_W_b.y() << '\n' << velocity_w_.z() << " " << velocity_W_b.z() << '\n');
        rate.sleep();
    }
    ros::spin();
    return 0;
}

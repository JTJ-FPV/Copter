#include <ros/ros.h>
#include <mavros_msgs/VFR_HUD.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

nav_msgs::Odometry odom;
double alpha;
// Eigen::Quaterniond q_wb;

gazebo_msgs::ModelStates model_state;

geometry_msgs::Twist velocity_w;

geometry_msgs::Pose p_wb;

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom = *msg;
    Eigen::Quaterniond q_wb(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    Eigen::Vector3d v_b(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
    // ROS_INFO_STREAM("odometry body : " << v_b);
    if(v_b.norm()!=0 && v_b.x() >= 9)   // 失速
    {
        v_b << v_b.x(), 0, v_b.z();
        double t1 = v_b.z() / v_b.x();
        double t2 = atan2(v_b.z(), v_b.x());

        ROS_INFO_STREAM("tan is " << t1);

        ROS_INFO_STREAM("arctan is " << t2);

        alpha = atan2(v_b.z(), v_b.x()) * 180 / M_PI; 
    }
    else
        alpha = 0;
    ROS_INFO_STREAM("the alpha is : " << alpha);
}

void gazeboCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    model_state = *msg;
    velocity_w = msg->twist.at(2);
    p_wb = msg->pose.at(2);
    Eigen::Quaterniond q_wb(p_wb.orientation.w, p_wb.orientation.x, p_wb.orientation.y, p_wb.orientation.z);
    Eigen::Vector3d v_w(velocity_w.linear.x, velocity_w.linear.y, velocity_w.linear.z), v_b;
    v_b = q_wb.conjugate() * v_w;
    if(v_w.norm()!=0 && v_b.x() >= 9)
    {
        v_b << v_b.x(), 0, v_b.z();

        // 归一化

        alpha = atan2(v_b.z(), v_b.x()) * 180 / M_PI; 

    }
    else
        alpha = 0;
    ROS_INFO_STREAM("the alpha is : " << alpha);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airspeed_test");
    ros::NodeHandle nh;
    ros::Subscriber sim_gazebo_pose_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, gazeboCallback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/odometry/in", 1, odomCallback, ros::TransportHints().tcpNoDelay());
    ros::Rate rate(60);
    rate.sleep();
    ros::spin();
    return 0;
}
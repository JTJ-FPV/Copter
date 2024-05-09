#include <ros/ros.h>
#include <mavros_msgs/VFR_HUD.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <Eigen/Core>

gazebo_msgs::ModelStates model_state;

geometry_msgs::Twist velocity_w;

mavros_msgs::VFR_HUD airspeed;

void gazeboCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    model_state = *msg;
    velocity_w = msg->twist.at(2);
}

void airspeedCallback(const mavros_msgs::VFR_HUDConstPtr &msg)
{
    airspeed = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airspeed_test");
    ros::NodeHandle nh;
    ros::Subscriber sim_gazebo_pose_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, gazeboCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber airspeed_sub = nh.subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud", 1, airspeedCallback, ros::TransportHints().tcpNoDelay());
    ros::Rate rate(20);
    while (ros::ok())
    {
        Eigen::Vector3d V_w(velocity_w.linear.x, velocity_w.linear.y, velocity_w.linear.z);
        ROS_INFO_STREAM("the Inertial Frame velocity,airspeed and groundspeed is :" << V_w.norm() << ", " << airspeed.airspeed << ", " << airspeed.groundspeed);
        ROS_INFO_STREAM("different : " << V_w.norm() - airspeed.groundspeed);
        rate.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

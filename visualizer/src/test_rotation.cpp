#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

geometry_msgs::PoseStamped pose;

void gazeboCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    pose.pose = msg->pose.at(2);
    Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    Eigen::Vector3d v(1, 1, 1);
    ROS_INFO_STREAM("the vector v (1, 1, 1) after rotation is :" << q * v);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::Subscriber sim_gazebo_pose_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, gazeboCallback, ros::TransportHints().tcpNoDelay());
    ros::spin();
    return 0;
}

#include <ros/ros.h>
#include <std_msgs/Float64.h>

std_msgs::Float64 signal_;

void signalCallback(const std_msgs::Float64ConstPtr &msg)
{
    signal_ = *msg;
    ROS_INFO_STREAM("signal is " << signal_.data);
}

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "adc_sub");
    ROS_INFO("node init");
    ros::NodeHandle nh;
    ROS_INFO("NodeHnadle");
    ros::Subscriber adc_sub = nh.subscribe<std_msgs::Float64>("adc/value", 1000, signalCallback);
    ROS_INFO("Subscriber");
    ros::spin();
    return 0;
}

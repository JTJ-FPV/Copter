#include "Visualizer.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualizer");
    ros::NodeHandle nh("~");
    CUADC::Visualizer visualizer(nh);
    ros::spin();
    return 0;
}

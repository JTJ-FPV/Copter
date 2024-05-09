#include <ros/ros.h>
#include "occupancy_grid_map/occ_grid_map.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occ_grid_map");
    ros::NodeHandle nh("~");
    CUADC::Map map;
    map.initMap(nh);
    ros::spin();
    return 0;
}

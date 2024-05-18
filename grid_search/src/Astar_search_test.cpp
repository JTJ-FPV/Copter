#include <ros/ros.h>
#include "grid_search/Astar_search.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/exact_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, geometry_msgs::PoseStamped> MySyncPolicy_odom_target;

// CUADC::Map map;
CUADC::MapPtr map_ptr;
CUADC::AstarPathFinder * astar_ptr;

ros::Publisher path_pub;
ros::Publisher path_pub_;

nav_msgs::Odometry odom;
Eigen::Vector3d start, target;
std::vector<Eigen::Vector3d> path;


pcl::PointCloud<pcl::PointXYZ> cloud_;

void publishPath(std::vector<Eigen::Vector3d> &path);

void visGridPath(std::vector<Eigen::Vector3d> &_path)
{
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = astar_ptr->map_->mp_.frame_id_;
    node_vis.header.stamp = ros::Time::now();

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;  // 立方体
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.5;
    node_vis.color.b = 1.0;

    node_vis.scale.x = astar_ptr->map_->mp_.resolution_;
    node_vis.scale.y = astar_ptr->map_->mp_.resolution_;
    node_vis.scale.z = astar_ptr->map_->mp_.resolution_;

    geometry_msgs::Point pt;
    for(auto &p:_path)
    {
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();

        node_vis.points.push_back(pt);
    }
    path_pub.publish(node_vis);
}

void odomCallback(const nav_msgs::OdometryConstPtr &_odom)
{
    odom = *_odom;
}

void targetCallback(const geometry_msgs::PoseStampedConstPtr &targ)
{
    path.clear();
    start << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    target << targ->pose.position.x, targ->pose.position.y, targ->pose.position.z;
    Eigen::Vector3i target_idx;
    astar_ptr->map_->pos2Index(target, target_idx);
    if(astar_ptr->map_->isOccupied(target_idx) == CUADC::VoxelState::OCCUPANY)
    {    
        ROS_WARN("the target is occupancied!");
        return;
    }
    ROS_INFO_STREAM("the target point is " << '\n' << target);
    astar_ptr->AstarGraphSearch(start, target, CUADC::AstarHeu::DIALOG_TIEBREAKER);
    astar_ptr->getPath(path);
    visGridPath(path);
    std::vector<Eigen::Vector3d> rdp_path;
    astar_ptr->rdpPath(path, 0.3, rdp_path);
    publishPath(rdp_path);
    ROS_INFO("find path");
    ROS_INFO_STREAM("the A* path size is " << path.size());
    ROS_INFO_STREAM("the rdp path size is " << rdp_path.size());
}

void publishPath(std::vector<Eigen::Vector3d> &path)
{
    if(path_pub_.getNumSubscribers() <= 0)
        return;
    cloud_.clear();
    pcl::PointXYZ pt;
    // Eigen::Vector3i id;
    // Eigen::Vector3d pos;
    for(auto &p:path)
    {
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        cloud_.push_back(pt);
    }
    cloud_.width = cloud_.points.size();
    cloud_.height = 1;
    cloud_.is_dense = true;
    cloud_.header.frame_id = map_ptr->mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud_, cloud_msg);
    path_pub_.publish(cloud_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_star");
    ros::NodeHandle nh("~");

    ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 1, targetCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/iris_0/mavros/odometry/in", 1, odomCallback, ros::TransportHints().tcpNoDelay());
    path_pub = nh.advertise<visualization_msgs::Marker>("a_star_path", 100);
    path_pub_ = nh.advertise<sensor_msgs::PointCloud2>("path_cloud", 100);
    CUADC::Map map;
    map.initMap(nh);
    map_ptr = &map;

    astar_ptr = new CUADC::AstarPathFinder();
    astar_ptr->initAstarPathFinder(&map);


    ros::spin();

    delete astar_ptr;
    delete map_ptr;

    return 0;
}

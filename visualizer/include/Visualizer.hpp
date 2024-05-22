#pragma once

#include "VisualizerConfig.hpp"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/Trajectory.h>
#include <trajectory_msgs/WayPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <vector>

namespace CUADC{
class Visualizer
{
public:

    enum Trajectory_type{
        WayPoint=0u,
        Planner_Trajectory=1u,
        PredictTrajectory=2u,
        HistoryTrajectory=3u
    };

private:
    ros::NodeHandle nh;
    /* ROS publisher */
    ros::Publisher robot_modelPub;
    ros::Publisher wayPointsPub;
    ros::Publisher planner_trajectoryPub;
    ros::Publisher history_trajectoryPub;
    ros::Publisher predict_trajectoryPub;

    /* ROS subscriber */
    ros::Subscriber sim_gazebo_pose_sub;
    ros::Subscriber vehicle_internal_pose_sub;
    ros::Subscriber vehicle_external_pose_sub;
    ros::Subscriber predict_trajectory_sub;
    ros::Subscriber planner_trajectory_sub;
    ros::Subscriber way_point_sub;
    
    /* ROS parameters */
    VisualizerConfig config;
    visualization_msgs::Marker robot_model;
    visualization_msgs::Marker way_points_marker; 
    visualization_msgs::Marker planner_traj_marker; trajectory_msgs::Trajectory planner_traj; bool planner_traj_sign;
    visualization_msgs::Marker predict_traj_marker; trajectory_msgs::Trajectory predict_traj;
    nav_msgs::Path history_traj;
    geometry_msgs::PoseStamped vehicle_pose, prior_vehicle_pose;
    trajectory_msgs::WayPoint way_point;
    /* ROS Timer event */
    ros::Timer FPV;
    tf::TransformBroadcaster map2baselink;

    /* ROS init functions */
    void Init_publisher();
    void Init_subscriber();
    void Init_paramters();

    /* ROS callback */
    void simGazeboPoseCallback(const gazebo_msgs::ModelStatesConstPtr &msg);
    void vehicleInternalPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void vehicleExternalPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void wayPointTrajGeneration(const geometry_msgs::PoseStamped &pose);
    void FPVCallback(const ros::TimerEvent &fpv);
    void PredictTrajCallback(const trajectory_msgs::TrajectoryConstPtr &traj);
    void TrajVectorGeneration(const std::vector<geometry_msgs::PoseStamped> &traj);
    void PlannerTrajCallback(const trajectory_msgs::TrajectoryConstPtr &traj);
    void WayPointCallback(const trajectory_msgs::WayPointConstPtr &w_p);

    /* other function */
    // _markers_type
    template<typename T>
    void CheckResetContainer(std::vector<T> &container, Trajectory_type type)
    {
        // ROS_ERROR("CheckResetContainer Point");
        // ROS_INFO_STREAM("the contain type is " << type << "size is " << container.size());
        long unsigned int size = 10;
        switch (type)
        {
        case 0u:
            size = config.waypoint.Contain;
            break;
        case 1u:
            size = config.trajectory.Contain;
            break;
        case 2U:
            size = config.predict_traj.Contain;
            break;
        case 3U:
            size = config.history_traj.Contain;
            break;
        }
        if(container.size() > size)
        {
            container.clear();
            // ROS_INFO_STREAM("The " << type << " trajectory is "<< size << " ,clear successfully");
        }
        if(Trajectory_type::Planner_Trajectory == type)
            planner_traj_sign = true;
    };
public:
    Visualizer(ros::NodeHandle &nh_):nh(nh_)
    {
        config.getParamters(nh);
        Init_paramters();
        Init_subscriber();
        Init_publisher();
    }

};

void Visualizer::Init_publisher()
{
    robot_modelPub = nh.advertise<visualization_msgs::Marker>(config.robot.Publish_RobotModelTopic, 1);
    wayPointsPub = nh.advertise<visualization_msgs::Marker>(config.waypoint.Publish_WayPointTopic, 1);
    planner_trajectoryPub = nh.advertise<visualization_msgs::Marker>(config.trajectory.Publish_TrajectoryTopic, 1);
    history_trajectoryPub = nh.advertise<nav_msgs::Path>(config.history_traj.HistoryTrajectoryTopic, 1);
    predict_trajectoryPub = nh.advertise<visualization_msgs::Marker>(config.predict_traj.Publish_PredictTrajectoryTopic, 1);
}

void Visualizer::Init_subscriber()
{
    predict_trajectory_sub = nh.subscribe<trajectory_msgs::Trajectory>(config.predict_traj.Subscribe_PredictTrajectoryTopic, 1, &Visualizer::PredictTrajCallback, this, ros::TransportHints().tcpNoDelay());
    planner_trajectory_sub = nh.subscribe<trajectory_msgs::Trajectory>(config.trajectory.Subscribe_TrajectoryTopic, 1, &Visualizer::PlannerTrajCallback, this, ros::TransportHints().tcpNoDelay());
    way_point_sub = nh.subscribe<trajectory_msgs::WayPoint>(config.waypoint.Subscribe_WayPointTopic, 1, &Visualizer::WayPointCallback, this, ros::TransportHints().tcpNoDelay());
    if(config.vehicle_pose.use_simulation)
        sim_gazebo_pose_sub = nh.subscribe<gazebo_msgs::ModelStates>(config.vehicle_pose.gazebo_pose_topic, 1, &Visualizer::simGazeboPoseCallback, this, ros::TransportHints().tcpNoDelay());
    else if(config.vehicle_pose.use_custom_pose_estimate)
        vehicle_external_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(config.vehicle_pose.px4_custom_pose_estimate_topic, 1, &Visualizer::vehicleExternalPoseCallback, this, ros::TransportHints().tcpNoDelay());
    else
        vehicle_internal_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(config.vehicle_pose.px4_pose_topic, 1, &Visualizer::vehicleInternalPoseCallback, this, ros::TransportHints().tcpNoDelay());

}

void Visualizer::Init_paramters()
{   
    /* 飞机模型参数初始化 */
    robot_model.type = visualization_msgs::Marker::MESH_RESOURCE;
    robot_model.action = visualization_msgs::Marker::ADD;
    robot_model.mesh_resource = config.robot.RobotModel;
    robot_model.mesh_use_embedded_materials = true;
    robot_model.header.frame_id = config.robot.RobotModel_frame;
    robot_model.ns = config.robot.RobotModel_ns;
    robot_model.id = config.robot.RobotModel_id;
    robot_model.pose.orientation.w = 1.0;
    robot_model.scale.x = config.robot.Scale_x;
    robot_model.scale.y = config.robot.Scale_y;
    robot_model.scale.z = config.robot.Scale_z;
    
    /* 规划器轨迹参数初始化 */
    planner_traj_marker.type = visualization_msgs::Marker::LINE_LIST;
    planner_traj_marker.action = visualization_msgs::Marker::ADD;
    planner_traj_marker.header.frame_id = config.trajectory.Trajectory_frame;
    planner_traj_marker.ns = config.trajectory.Publish_TrajectoryTopic;
    planner_traj_marker.id = config.trajectory.Trajectory_id;
    planner_traj_marker.scale.x = config.trajectory.Scale_x;
    planner_traj_marker.color.r = config.trajectory.Color_r;
    planner_traj_marker.color.g = config.trajectory.Color_g;
    planner_traj_marker.color.b = config.trajectory.Color_b;
    planner_traj_marker.color.a = config.trajectory.Color_a;
    planner_traj_marker.pose.orientation.w = 1.0;
    planner_traj_sign = true;
    // predict_traj_marker_container.markers.resize(config.predict_traj.Contain);


    /* 抛线预测轨迹 */
    predict_traj_marker.type = visualization_msgs::Marker::LINE_LIST;
    predict_traj_marker.action = visualization_msgs::Marker::ADD;
    predict_traj_marker.header.frame_id = config.predict_traj.PredictTrajectory_frame;
    predict_traj_marker.ns = config.predict_traj.PredictTrajectory_ns;
    predict_traj_marker.id = config.predict_traj.PredictTrajectory_id;
    predict_traj_marker.scale.x = config.predict_traj.Scale_x;
    predict_traj_marker.color.r = config.predict_traj.Color_r;
    predict_traj_marker.color.g = config.predict_traj.Color_g;
    predict_traj_marker.color.b = config.predict_traj.Color_b;
    predict_traj_marker.color.a = config.predict_traj.Color_a;
    predict_traj_marker.pose.orientation.w = 1.0;

    /* 航点参数初始化 */
    way_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    way_points_marker.action = visualization_msgs::Marker::ADD;
    way_points_marker.header.frame_id = config.waypoint.WayPoint_frame;
    way_points_marker.ns = config.waypoint.WayPoint_ns;
    way_points_marker.id = config.waypoint.WayPoint_id;
    way_points_marker.scale.x = config.waypoint.Scale_x;
    way_points_marker.scale.y = config.waypoint.Scale_y;
    way_points_marker.scale.z = config.waypoint.Scale_z;
    way_points_marker.color.r = config.waypoint.Color_r;
    way_points_marker.color.g = config.waypoint.Color_g;
    way_points_marker.color.b = config.waypoint.Color_b;
    way_points_marker.color.a = config.waypoint.Color_a;
    way_points_marker.pose.orientation.w = 1.0;

    /* 历史轨迹参数初始化 */
    history_traj.header.frame_id = config.history_traj.HistoryTrajectory_frame;

     /* 第一视角 */
    if(config.frame.useFPV)
        FPV = nh.createTimer(ros::Duration(ros::Rate(config.Rate)), &Visualizer::FPVCallback, this);
}

void Visualizer::simGazeboPoseCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    geometry_msgs::PoseStamped pose;
    pose.pose = msg->pose.at(2);
    // pose.pose = msg->pose.at(13);
    wayPointTrajGeneration(pose);
}

void Visualizer::vehicleInternalPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    geometry_msgs::PoseStamped pose = *msg;
    wayPointTrajGeneration(pose);
}

void Visualizer::vehicleExternalPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    geometry_msgs::PoseStamped pose = *msg;
    wayPointTrajGeneration(pose);
}

void Visualizer::wayPointTrajGeneration(const geometry_msgs::PoseStamped &pose)
{
    vehicle_pose = pose;
    robot_model.header.stamp = planner_traj_marker.header.stamp = way_points_marker.header.stamp = history_traj.header.stamp = ros::Time::now();
    robot_model.pose = pose.pose;
    robot_model.lifetime = planner_traj_marker.lifetime = predict_traj_marker.lifetime = ros::Duration(1);
    if(config.trajectory.History_Trajectory_Marker)    // 使用Marker显示历史轨迹
    {
        if(!planner_traj_sign)
        {
            prior_vehicle_pose = vehicle_pose;
            vehicle_pose = pose;
        }
        else
        {
            prior_vehicle_pose = vehicle_pose;
            planner_traj_sign = false;
        }
        CheckResetContainer(planner_traj_marker.points, Trajectory_type::Planner_Trajectory);
        planner_traj_marker.points.push_back(prior_vehicle_pose.pose.position);
        planner_traj_marker.points.push_back(vehicle_pose.pose.position);
        planner_trajectoryPub.publish(planner_traj_marker);
    }
    /* 发布机器人模型 */
    robot_modelPub.publish(robot_model);
    // ROS_INFO_STREAM("the model pose is " << robot_model.pose);

    /* 发布历史轨迹 */
    CheckResetContainer(history_traj.poses, Trajectory_type::HistoryTrajectory);
    history_traj.poses.push_back(pose);
    history_traj.poses.push_back(pose);
    history_trajectoryPub.publish(history_traj);
    ros::Duration(1 / config.Rate).sleep();
}

void Visualizer::FPVCallback(const ros::TimerEvent &fpv)
{
    map2baselink.sendTransform(tf::StampedTransform(tf::Transform(
      tf::Quaternion(vehicle_pose.pose.orientation.x, vehicle_pose.pose.orientation.y, vehicle_pose.pose.orientation.z, vehicle_pose.pose.orientation.w), 
      tf::Vector3(vehicle_pose.pose.position.x, vehicle_pose.pose.position.y, vehicle_pose.pose.position.z)
    ), ros::Time::now(), config.frame.OriginFrame, config.frame.VehicleFrame));
    ros::Duration(1 / config.Rate).sleep();
    
    wayPointsPub.publish(way_points_marker);

    planner_trajectoryPub.publish(planner_traj_marker);
    CheckResetContainer(planner_traj_marker.points, Trajectory_type::Planner_Trajectory);

}

void Visualizer::PredictTrajCallback(const trajectory_msgs::TrajectoryConstPtr &msg)
{
    predict_traj = *msg;
    predict_traj_marker.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped last, current;
    bool predict_sign = true;
    for(auto &i : predict_traj.traj)
    {
        if(predict_sign)
        {  
            last = i;
            current = i;
            predict_sign = false;
        }
        else
        {
            last = current;
            current = i;
            predict_traj_marker.points.push_back(last.pose.position);
            predict_traj_marker.points.push_back(current.pose.position);
        }
    }
    predict_trajectoryPub.publish(predict_traj_marker);
    CheckResetContainer(predict_traj_marker.points, Trajectory_type::PredictTrajectory);
    ros::Duration(1 / config.Rate).sleep();
} // class Visualizer

void Visualizer::WayPointCallback(const trajectory_msgs::WayPointConstPtr &w_p)
{
    way_point = *w_p;
    way_points_marker.header.stamp = ros::Time::now();
    // for(int i = 0; i < way_point.waypoint.size(); ++i)
    for(auto &p:way_point.waypoint)
    {
        CheckResetContainer(way_points_marker.points, Trajectory_type::WayPoint);
        /* 发布航点 */
        geometry_msgs::Point point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        way_points_marker.points.push_back(point);
    }
}

void Visualizer::PlannerTrajCallback(const trajectory_msgs::TrajectoryConstPtr &traj)
{
    planner_traj = *traj;
    planner_traj_marker.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped last, current;
    bool planner_sign = true;
    ROS_INFO_STREAM("the planner traj size is " << planner_traj.traj.size());
    for(auto &i : planner_traj.traj)
    {
        if(planner_sign)
        {  
            last = i;
            current = i;
            planner_sign = false;
        }
        else
        {
            last = current;
            current = i;
            planner_traj_marker.points.push_back(last.pose.position);
            planner_traj_marker.points.push_back(current.pose.position);
        }
    }
    // planner_trajectoryPub.publish(planner_traj_marker);
    // CheckResetContainer(planner_traj_marker.points, Trajectory_type::Planner_Trajectory);
    // ros::Duration(1 / config.Rate).sleep();
}

} // namespace CUADC
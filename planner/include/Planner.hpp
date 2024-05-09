#pragma once

/* eigen3 线性代数库 */
#include <Eigen/Core>
#include <Eigen/Geometry>

/* configure */
#include "PlannerConfig.hpp"


/* 多消息同步回调 */
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

/* message type*/
#include <trajectory_msgs/Trajectory.h>
#include <trajectory_msgs/WayPoint.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>

/*std*/
#include <vector>

namespace CUADC{

class Planner
{
private:
    ros::NodeHandle nh;
    /* ROS publisher */
    ros::Publisher  planner_trajectoryPub;      // 发布规划出的曲线
    ros::Publisher  planner_miniJerk_waypointPub;        // 发布 minimumJerk 航点
    ros::Publisher  parabola_trajectoryPub;     // 发布计算出的抛线  
    ros::Publisher  vision_positionPub;
    
    geometry_msgs::PoseStamped home_position;   // 起飞点   
    geometry_msgs::PoseStamped vision_position;   // 视觉识别到的点（世界坐标系下）
    bool home_position_sign, multiCopter_vision_sign, receive_vision_position_sign; long unsigned int multiCopter_step;
public:
    ros::Publisher  controlPub;                 // 发布无人机控制信号

private:
    /* ROS subscriber */
    ros::Subscriber sim_gazebo_pose_sub;
    ros::Subscriber px4_odom_sub;
    ros::Subscriber px4_state_sub;
    ros::Subscriber home_position_sub;
    ros::Subscriber vision_position_sub;
    typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::Imu> MySyncPolicy_odom_imu;
    
    /* ROS parameters */
    /* first position, current position */                                       
    mavros_msgs::PositionTarget first_position, current_position, multicopter_task_position;   uint32_t fp_count;      
public:
    /* px4 state */
    mavros_msgs::State current_state;
private:
    /* Planner Config */        
    PlannerConfig config;       
public:
    /* 位姿 */
    geometry_msgs::Pose vehicle_pose;
private:
    /* Planner Trajectory*/
    trajectory_msgs::Trajectory planner_trajectory;         trajectory_msgs::WayPoint way_point;    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> internal_point;
    Eigen::VectorXd time;   Eigen::Matrix3Xd positions;     bool minijerk_control, minijerk_traj_flash;  // minijerk_control: true 表示可以控制，false表示不可能控制    minijerk_traj_flash true 表示可以生成minimumJerk轨迹，false表示不生成轨迹
    u_int32_t positionNum;
    /* Parabola Trajectory paramters*/
    trajectory_msgs::Trajectory parabola_trajectory;
    double parabola_T;
    /* Control Config */
    mavros_msgs::PositionTarget control_point; uint64_t control_count;

public:
    std::vector<mavros_msgs::PositionTarget> minijerk_trajectory;       // minijerk轨迹用于外部控制

private:
    //PID
    double KP_Z = 1.5, KI_Z = 0.007, KD_Z = 0.4;
    double KP_Y = 1.5, KI_Y = 0.001, KD_Y = 0.5;
    double KP_X = 1.5, KI_X = 0.001, KD_X = 0.4;

	double err_x = 0;     double err_last_x = 0;		double err_next_x = 0;
	double err_y = 0;     double err_last_y = 0;		double err_next_y = 0;
	double err_z = 0;     double err_last_z = 0;		double err_next_z = 0;
	double increase_vel_x, increase_vel_y, increase_vel_z;
    geometry_msgs::Twist exp_vel;

    /* ROS Timer event */
    ros::Timer control; // miniJerk
    ros::Timer multiCopter_control;

    /* ROS init functions */
    void Init_paramters();
    void Init_subscriber();
    void Init_publisher();

    /* ROS callback */
    void simGazeboPoseCallback(const gazebo_msgs::ModelStatesConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void MultiCopter_odom_ImuCallback(const nav_msgs::OdometryConstPtr &vehicle_pose, const sensor_msgs::ImuConstPtr &imu_data);
    void Fixedwing_NMPC_odom_ImuCallback(const nav_msgs::OdometryConstPtr &vehicle_pose, const sensor_msgs::ImuConstPtr &imu_data);
    void controlCallback(const ros::TimerEvent &control);
    void px4StateCallback(const mavros_msgs::StateConstPtr &msg);
    void MulticopterControl(const ros::TimerEvent&);
    void homePositionCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void visionPositionCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    /* other function */
    double Traj_X(const double &t, const Eigen::Vector3d &V_w, const Eigen::Vector3d &P_w, const PlannerConfig &config);
    double Traj_Y(const double &t, const Eigen::Vector3d &V_w, const Eigen::Vector3d &P_w, const PlannerConfig &config);
    double Traj_Z(const double &t, const Eigen::Vector3d &V_w, const Eigen::Vector3d &P_w, const PlannerConfig &config);
    void GenerationParabolaTrajectory(const Eigen::Vector3d &P_w, const Eigen::Vector3d &V_w, const PlannerConfig &config, trajectory_msgs::Trajectory &parabola_trajectory);
    void GenerationMinimumJerkTrajectory(const Eigen::Vector3d &P_w, const Eigen::Vector3d &V_w, const Eigen::Vector3d &Acc_w, const PlannerConfig &config);
    double getParabolaTime(){return parabola_T;}
    void PidVelocityControl(const Eigen::Vector3d &p, mavros_msgs::PositionTarget &control);
    /* 轨迹平滑 */
    void minimumJerkTrajGen(
    // Inputs:
    const int pieceNum,
    const Eigen::Vector3d &initialPos,
    const Eigen::Vector3d &initialVel,
    const Eigen::Vector3d &initialAcc,
    const Eigen::Vector3d &terminalPos,
    const Eigen::Vector3d &terminalVel,
    const Eigen::Vector3d &terminalAcc,
    const Eigen::Matrix3Xd &intermediatePositions,
    const Eigen::VectorXd &timeAllocationVector,
    // Outputs:
    Eigen::MatrixX3d &coefficientMatrix);
    /* 计算两个路标点的时间 */
    double timeTraj(const double dist, const double vel, const double acc);
    void TimeMatrix(double t, Eigen::Matrix<double, 3, 6> &time_matrix);
    bool checkFirstPosition();
    bool checkPosition(const Eigen::Vector3d p, const double accuracy);
public:
    Planner(ros::NodeHandle &nh_):nh(nh_)
    {
        config.getParam(nh);
        Init_paramters();
        Init_subscriber();
        Init_publisher();
    }
};

void Planner::Init_paramters()
{
    home_position_sign = false;
    multiCopter_vision_sign = false;
    receive_vision_position_sign = false;
    multiCopter_step = 0;
    
    planner_trajectory.Header.frame_id = config.planner_trajectory.PlannerTrajectory_frame;
    parabola_trajectory.Header.frame_id = config.parabola_trajectory.ParabolaTrajectory_frame;

    way_point.Header.frame_id = config.planner_trajectory.PlannerTrajectory_frame;
    positionNum = config.planner_trajectory.MinimumJerk_WayPoint.size() / 3 + 1;
    time.resize(positionNum - 1);
    positions.resize(3, positionNum);
    minijerk_control = false;    // 不允许控制
    minijerk_traj_flash = true;  // 不允许生成minimumJerk轨迹
    for(uint32_t i = 0; i < config.planner_trajectory.MinimumJerk_WayPoint.size() / 3; ++i)
    {
        Eigen::Vector3d point(config.planner_trajectory.MinimumJerk_WayPoint.at(i * 3), config.planner_trajectory.MinimumJerk_WayPoint.at(i * 3 + 1), config.planner_trajectory.MinimumJerk_WayPoint.at(i * 3 + 2));
        internal_point.push_back(point);
    }


    /* control paramters */
    control_count = 0;
    control_point.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    control_point.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
                            //  |mavros_msgs::PositionTarget::IGNORE_PX |
                            //   mavros_msgs::PositionTarget::IGNORE_PY |
                            //   mavros_msgs::PositionTarget::IGNORE_PZ;

    first_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    first_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    first_position.position.x = config.planner_trajectory.FirstPoint.at(0);
    first_position.position.y = config.planner_trajectory.FirstPoint.at(1);
    first_position.position.z = config.planner_trajectory.FirstPoint.at(2);
    fp_count = 0;   // firstPosition计数器

    current_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                                 mavros_msgs::PositionTarget::IGNORE_VY |
                                 mavros_msgs::PositionTarget::IGNORE_VZ |
                                 mavros_msgs::PositionTarget::IGNORE_AFX |
                                 mavros_msgs::PositionTarget::IGNORE_AFY |
                                 mavros_msgs::PositionTarget::IGNORE_AFZ |
                                 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    multicopter_task_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    multicopter_task_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                                 mavros_msgs::PositionTarget::IGNORE_VY |
                                 mavros_msgs::PositionTarget::IGNORE_VZ |
                                 mavros_msgs::PositionTarget::IGNORE_AFX |
                                 mavros_msgs::PositionTarget::IGNORE_AFY |
                                 mavros_msgs::PositionTarget::IGNORE_AFZ |
                                 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
}

void Planner::Init_subscriber()
{
    /* 同时订阅位姿与加速度计数据 */
    if(config.planner_trajectory.use_MinimumJerk && config.planner_trajectory.MultiCopterPlanner && !config.fixedwing_planner.use_MpcPlanner)
    {
        static message_filters::Subscriber<sensor_msgs::Imu> imu_data(nh, config.vehicle_pose.vehicle_imu_topic, 1, ros::TransportHints().tcpNoDelay());
        static message_filters::Subscriber<nav_msgs::Odometry> odom_data(nh, config.vehicle_pose.px4_odom_topic, 1, ros::TransportHints().tcpNoDelay());
        static message_filters::Synchronizer<MySyncPolicy_odom_imu> sync_odom_imu(MySyncPolicy_odom_imu(1), odom_data, imu_data);
        sync_odom_imu.registerCallback(boost::bind(&Planner::MultiCopter_odom_ImuCallback, this, _1, _2));
        ROS_INFO("MinimumJerk MultiCopter");
    }
    else if(config.fixedwing_planner.use_MpcPlanner)
    {
        static message_filters::Subscriber<sensor_msgs::Imu> imu_data(nh, config.vehicle_pose.vehicle_imu_topic, 1, ros::TransportHints().tcpNoDelay());
        static message_filters::Subscriber<nav_msgs::Odometry> odom_data(nh, config.vehicle_pose.px4_odom_topic, 1, ros::TransportHints().tcpNoDelay());
        static message_filters::Synchronizer<MySyncPolicy_odom_imu> sync_odom_imu(MySyncPolicy_odom_imu(1), odom_data, imu_data);
        sync_odom_imu.registerCallback(boost::bind(&Planner::Fixedwing_NMPC_odom_ImuCallback, this, _1, _2));
        ROS_INFO("NMPC Fixed-Wing");
    }
    else
    {
        ROS_INFO("No Planner Init, Please check the yaml");
        // ROS_BREAK();
    }
    if(config.vehicle_pose.use_simulation_pose)
        sim_gazebo_pose_sub = nh.subscribe<gazebo_msgs::ModelStates>(config.vehicle_pose.gazebo_pose_topic, 1, &Planner::simGazeboPoseCallback, this, ros::TransportHints().tcpNoDelay());
    else
        px4_odom_sub = nh.subscribe<nav_msgs::Odometry>(config.vehicle_pose.px4_odom_topic, 1, &Planner::odomCallback, this, ros::TransportHints().tcpNoDelay());
    px4_state_sub = nh.subscribe<mavros_msgs::State>(config.vehicle_pose.px4_state_topic, 1, &Planner::px4StateCallback, this, ros::TransportHints().tcpNoDelay());
    home_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/CUADC/HomePosition", 1, &Planner::homePositionCallback, this, ros::TransportHints().tcpNoDelay());
    vision_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(config.planner_trajectory.Subscribe_TargetPoint_World_FLU_Topic, 10, &Planner::visionPositionCallback, this, ros::TransportHints().tcpNoDelay());
}

void Planner::Init_publisher()
{
    parabola_trajectoryPub = nh.advertise<trajectory_msgs::Trajectory>(config.parabola_trajectory.Publish_ParabolaTrajectoryTopic, 1);
    planner_trajectoryPub = nh.advertise<trajectory_msgs::Trajectory>(config.planner_trajectory.Publish_PlannerTopic, 1);
    if(config.planner_trajectory.MultiCopterPlanner)
    {
        planner_miniJerk_waypointPub = nh.advertise<trajectory_msgs::WayPoint>(config.planner_trajectory.PublishMinimumJerk_WayPointTopic, 1);
        controlPub = nh.advertise<mavros_msgs::PositionTarget>(config.control.Control_topic, 1);
        vision_positionPub = nh.advertise<std_msgs::Bool>("/camera/target/world/point/sign", 1);
        if(!config.planner_trajectory.MultiCopterControl)
            control = nh.createTimer(ros::Duration(ros::Rate(config.control.Control_Rate)), &Planner::controlCallback, this);
        else
            multiCopter_control = nh.createTimer(ros::Duration(ros::Rate(config.control.Control_Rate)), &Planner::MulticopterControl, this);
    }    
}

void Planner::simGazeboPoseCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    geometry_msgs::Pose pose = msg->pose.at(2);     vehicle_pose = pose;
    current_position.position = pose.position;
    geometry_msgs::Twist velocity_w = msg->twist.at(2);
    Eigen::Vector3d P_w(pose.position.x, pose.position.y, pose.position.z), V_w(velocity_w.linear.x, velocity_w.linear.y, velocity_w.linear.z);
    GenerationParabolaTrajectory(P_w, V_w, config, parabola_trajectory);
    ros::Duration(1 / config.Rate).sleep();
}

void Planner::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;      vehicle_pose = pose;
    current_position.position = pose.position;
    geometry_msgs::Twist velocity_w = msg->twist.twist;
    Eigen::Vector3d P_w(pose.position.x, pose.position.y, pose.position.z), V_b(velocity_w.linear.x, velocity_w.linear.y, velocity_w.linear.z), V_w;
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    V_w = q * V_b;
    GenerationParabolaTrajectory(P_w, V_w, config, parabola_trajectory);
    ros::Duration(1 / config.Rate).sleep();
}

void Planner::MultiCopter_odom_ImuCallback(const nav_msgs::OdometryConstPtr &vehicle_pose, const sensor_msgs::ImuConstPtr &imu_data)
{
    /* 获取位姿数据 */
    geometry_msgs::Pose pose = vehicle_pose->pose.pose;         this->vehicle_pose = pose;
    current_position.position = pose.position;
    geometry_msgs::Twist velocity_b = vehicle_pose->twist.twist;
    Eigen::Vector3d P_w(pose.position.x, pose.position.y, pose.position.z), V_b(velocity_b.linear.x, velocity_b.linear.y, velocity_b.linear.z), V_w;
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    V_w = q * V_b;      // 将速度转换成世界坐标系下的速度
    
    /* 获取加速度数据 */
    sensor_msgs::Imu imu = *imu_data;
    Eigen::Vector3d acc_b(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z), acc_w;
    acc_w = q * acc_b;      // 将机体坐标下的加速度转换成世界坐标系下的加速度

    if(minijerk_traj_flash)
        GenerationMinimumJerkTrajectory(P_w, V_w, acc_w, config);
    ros::Duration(1 / config.Rate).sleep();
}

void Planner::Fixedwing_NMPC_odom_ImuCallback(const nav_msgs::OdometryConstPtr &vehicle_pose, const sensor_msgs::ImuConstPtr &imu_data)
{
    geometry_msgs::Pose pose = vehicle_pose->pose.pose;         this->vehicle_pose = pose;
    current_position.position = pose.position;
    geometry_msgs::Twist velocity_b = vehicle_pose->twist.twist;
    Eigen::Vector3d P_w(pose.position.x, pose.position.y, pose.position.z), V_b(velocity_b.linear.x, velocity_b.linear.y, velocity_b.linear.z), V_w;
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    V_w = q * V_b;      // 将速度转换成世界坐标系下的速度

}

void Planner::controlCallback(const ros::TimerEvent &control)
{   
    if(!config.planner_trajectory.MultiCopterControl)
    {
        if(current_state.connected)
        {        
            if(fp_count < 100)
            {
                controlPub.publish(first_position);
                fp_count++;
            }
            if(current_state.mode == "OFFBOARD" && current_state.armed)
            {   
                if(minijerk_control)
                {
                    if(!minijerk_trajectory.empty() && control_count < minijerk_trajectory.size())
                    {
                        controlPub.publish(minijerk_trajectory.at(control_count++));
                    }
                    else if(!minijerk_trajectory.empty())
                    {
                        minijerk_control = false;  // minimumJerk不可以控制
                        controlPub.publish(minijerk_trajectory.at(control_count - 1));
                    }
                }
                else if(!minijerk_trajectory.empty())
                {
                    ROS_INFO("hold_position");
                    controlPub.publish(minijerk_trajectory.at(control_count - 1));
                }
                else
                {
                    ROS_INFO("first_position");
                    if(minijerk_trajectory.empty())
                    {
                        controlPub.publish(first_position);
                        if(checkFirstPosition())
                            minijerk_traj_flash = true;
                    }
                    else
                    {
                        controlPub.publish(current_position);
                    }
                }
            }
            else
            {
                controlPub.publish(first_position);
                ROS_INFO("waiting for offboard mode");
            }
        }
        else
            ROS_INFO("waiting for connect");
    }
    else
    {
        ROS_ERROR("Set the MultiCopterControl false!!!");
        ROS_BREAK();
    }
}

void Planner::px4StateCallback(const mavros_msgs::StateConstPtr &msg)
{
    current_state = *msg;
}

/*
 *  V_w 为世界坐标系下飞机的三轴速度，P_w 为世界坐标系下飞机的坐标，世界坐标系的原点为飞机上电时的位置，方向为FLU（前左天），
 *
*/
double Planner::Traj_X(const double &t, const Eigen::Vector3d &V_w, const Eigen::Vector3d &P_w, const PlannerConfig &config)
{
    double m = config.vehicle_paramters.m, k = config.vehicle_paramters.k;
    double X;
    if(V_w.x() > 0)
    {
        double C3 = -m / (k * V_w.x());
        double C4 = k * P_w.x() / m -log(std::fabs(-C3));
        X = (m / k ) * log(std::fabs(t - C3)) + m * C4 / k;
    }
    else
    {
        double C12 = 1 / V_w.x();
        double C13 = m * log(std::fabs(C12)) / k + P_w.x();
        X = - m * log(std::fabs(C12 - k * t / m)) / k + C13;
    }
    // if(V_w.x() > 0)
    // {
    //     double C3 = -m / (k * V_w.x());
    //     double C4 = k * P_w.x() / m -log(std::fabs(-C3));
    //     X = (m / k ) * log(std::fabs(t - C3)) + m * C4 / k;
    // }
    // else
    // {
    //     double C12 = 1 / V_w.x();
    //     double C13 = m * log(std::fabs(C12)) / k + P_w.x();
    //     X = - m * log(std::fabs(C12 - k * t / m)) / k + C13;
    // }
    return X;
}

double Planner::Traj_Y(const double &t, const Eigen::Vector3d &V_w, const Eigen::Vector3d &P_w, const PlannerConfig &config)
{
    double m = config.vehicle_paramters.m, k = config.vehicle_paramters.k;
    double Y;
    if(V_w.y() > 0)
    {
        double C5 = - m / (k * V_w.y());
        double C6 = k * P_w.y() / m -log(std::fabs(-C5));
        Y = (m / k ) * log(std::fabs(t - C5)) + m * C6 / k;
    }
    else
    {
        double C14 = 1 / V_w.y();
        double C15 = m * log(std::fabs(C14)) / k + P_w.y();
        Y = -m * log(std::fabs(C14 - k * t / m)) / k + C15;
    }
    return Y;
}

double Planner::Traj_Z(const double &t, const Eigen::Vector3d &V_w, const Eigen::Vector3d &P_w, const PlannerConfig &config)
{
    double m = config.vehicle_paramters.m, k = config.vehicle_paramters.k, g = config.vehicle_paramters.g;
    double Z = P_w.z();
    double a = -sqrt(m/(k*g));
    double b = sqrt(k/(m*g));
    // bug 在t>C1 时初值有问题
    if(V_w.z() > 0)
    {
        double C1 = -a * atan(b * V_w.z());
        double C2 = a * log(std::fabs(cos(C1 / a))) / b + P_w.z();
        if(t <= C1)
        {
            Z = -(a/b) * log(std::fabs(cos((t - C1) / a))) + C2;
        }
        else
        {
            double u = 1 / b;
            double Z_c1 = C2;
            double C10 = C1;
            double C11 = Z_c1 -  a * u * M_LN2;
            Z = u * (t - C10) + a * u * log(exp(-2 * (t - C10) / a) + 1) + C11;
        }
    }
    else
    {
        double u = 1 / b;
        if(-V_w.z() <= u)
        {
            Z = u * t + a * u * log(std::fabs((V_w.z() - u) / (V_w.z() + u)) * exp(-2 * t / a) + 1) - a * u * log(std::fabs((V_w.z() - u) / (V_w.z() + u)) + 1) + P_w.z();
        }
        else if(-V_w.z() > u)
        {
            Z = u * t + a * u * log(std::fabs((V_w.z() - u) / (V_w.z() + u)) * exp(-2 * t / a) - 1) - a * u * log(std::fabs((V_w.z() - u) / (V_w.z() + u)) - 1) + P_w.z();
        }
    }
    return Z;
}

void Planner::GenerationParabolaTrajectory(const Eigen::Vector3d &P_w, const Eigen::Vector3d &V_w, const PlannerConfig &config, trajectory_msgs::Trajectory &parabola_trajectory)
{
    parabola_trajectory.traj.clear();
    double Z = P_w.z(), X, Y;   int i = 1;
    double dt = config.vehicle_paramters.dt;
    geometry_msgs::PoseStamped traj_point;
    parabola_trajectory.Header.stamp = ros::Time::now();
    while(Z > config.parabola_trajectory.Parabola_terminal_high)
    {
        X = Traj_X(i * dt, V_w, P_w, config);
        Y = Traj_Y(i * dt, V_w, P_w, config);
        Z = Traj_Z(i * dt, V_w, P_w, config);
        parabola_T = i * dt; i++;
        traj_point.pose.position.x = X;
        traj_point.pose.position.y = Y;
        traj_point.pose.position.z = Z;
        parabola_trajectory.traj.push_back(traj_point);
    }
    // ROS_INFO_STREAM("The fly time is : " << parabola_T);
    // ROS_INFO_STREAM("the final position is : " << traj_point.pose.position);
    parabola_trajectoryPub.publish(parabola_trajectory);
}

double Planner::timeTraj(const double dist, const double vel, const double acc)
{
    const double t = vel / acc;
    const double d = 0.5 * acc * t * t;

    if (dist < d + d)
    {
        return sqrt(2.0 * dist / acc);
    }
    else
    {
        return 2.0 * t + (dist - 2.0 * d) / vel;
    }
}

void Planner::GenerationMinimumJerkTrajectory(const Eigen::Vector3d &P_w, const Eigen::Vector3d &V_w, const Eigen::Vector3d &Acc_w, const PlannerConfig &config)
{
    minijerk_traj_flash = false;
    positions.resize(3, positionNum);
    time.resize(positionNum - 1, 1);
    positions.block<3, 1>(0, 0) = P_w;      geometry_msgs::Point w_p;   way_point.waypoint.clear();
    way_point.Header.stamp = ros::Time::now();
    for(u_int32_t i = 1U; i < positionNum; ++i)
    {
        Eigen::Vector3d waypoint(config.planner_trajectory.MinimumJerk_WayPoint.at((i - 1)*3), config.planner_trajectory.MinimumJerk_WayPoint.at((i - 1)*3 + 1), config.planner_trajectory.MinimumJerk_WayPoint.at((i - 1)*3 + 2));
        positions.block<3, 1>(0, i) = waypoint;
        const double dist = (positions.col(i - 1) - positions.col(i)).norm();       // 两点的距离
        time(i - 1) = timeTraj(dist, config.vehicle_paramters.Max_v, config.vehicle_paramters.Max_a);
        w_p.x = waypoint.x(), w_p.y = waypoint.y(), w_p.z = waypoint.z();
        way_point.waypoint.push_back(w_p);
        ROS_INFO_STREAM(waypoint);
    }
    ROS_INFO("loop");
    planner_miniJerk_waypointPub.publish(way_point);
    const uint32_t pieceNum = positionNum - 1;
    const Eigen::Vector3d initialPos = positions.col(0);
    const Eigen::Vector3d initialVel = V_w;
    const Eigen::Vector3d initialAcc = Acc_w;
    ROS_INFO("initial");
    const Eigen::Vector3d terminalPos = positions.col(pieceNum);
    const Eigen::Vector3d terminalVel(config.planner_trajectory.TerminalVelocity.at(0), config.planner_trajectory.TerminalVelocity.at(1), config.planner_trajectory.TerminalVelocity.at(2));
    const Eigen::Vector3d terminalAcc(config.planner_trajectory.TerminalAccerlatation.at(0), config.planner_trajectory.TerminalAccerlatation.at(1), config.planner_trajectory.TerminalAccerlatation.at(2));
    ROS_INFO("terminal");
    ROS_INFO_STREAM("pieceNum - 1 = " << pieceNum - 1);
    ROS_INFO_STREAM(positions.middleCols(1, pieceNum - 1));
    const Eigen::Matrix3Xd intermediatePositions = positions.middleCols(1, pieceNum - 1);       // 中间点
    ROS_INFO("intermediatePositions");
    const Eigen::VectorXd timeAllocationVector = time.head(pieceNum);
    /* 五次多项式的轨迹参数 */
    Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6 * pieceNum, 3);
    ROS_INFO("minimumJerkTrajGen");
    minimumJerkTrajGen(pieceNum,
                    initialPos, initialVel, initialAcc,
                    terminalPos, terminalVel, terminalAcc,
                    intermediatePositions,
                    timeAllocationVector,
                    coefficientMatrix);
    /* 轨迹离散化 */
    planner_trajectory.traj.clear();       // 清空轨迹控制
    planner_trajectory.Header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped traj_point;
    Eigen::Matrix<double, 3, 6> time_matrix;
    Eigen::Matrix3d traj_state; double dt = config.vehicle_paramters.dt;
    /* 生成控制信号 */
    minijerk_trajectory.clear();        // 清空轨迹控制
    for(uint32_t i = 0; i < pieceNum; ++i)
    {
        uint32_t count = 0;
        // ROS_INFO_STREAM("the Time(" << i << ") : is " << '\n' << time(i));
        while(count * dt <= time(i))
        {
            /* 获得位置、速度、加速度 */
            TimeMatrix((count++) * dt, time_matrix);
            traj_state = time_matrix * coefficientMatrix.block<6, 3>(6 * i, 0);
            
            /* 轨迹离散化 */
            traj_point.pose.position.x = traj_state(0, 0);
            traj_point.pose.position.y = traj_state(0, 1);
            traj_point.pose.position.z = traj_state(0, 2);
            planner_trajectory.traj.push_back(traj_point);

            /* 生成控制信号 */
            /* 位置 */
            control_point.position.x = traj_state(0, 0);
            control_point.position.y = traj_state(0, 1);
            control_point.position.z = traj_state(0, 2);
            /* 速度 */
            control_point.velocity.x = traj_state(1, 0);
            control_point.velocity.y = traj_state(1, 1);
            control_point.velocity.z = traj_state(1, 2);
            /* 加速度 */
            control_point.acceleration_or_force.x = traj_state(2, 0);
            control_point.acceleration_or_force.y = traj_state(2, 1);
            control_point.acceleration_or_force.z = traj_state(2, 2);

            if((count + 1) * dt < time(i))
            {
                TimeMatrix((count+1) * dt, time_matrix);
                Eigen::Matrix3d traj_state_next = time_matrix * coefficientMatrix.block<6, 3>(6 * i, 0);
                Eigen::Matrix3d yaw = traj_state_next - traj_state;
                 ROS_INFO_STREAM('\n' << yaw);
                control_point.yaw = atan2f32(yaw(0, 1), yaw(0, 0));
                // ROS_INFO_STREAM("yaw is " << yaw(0, 1) << ',' << yaw(0, 0));
            }

            minijerk_trajectory.push_back(control_point);
        }
    }
    minijerk_control = true;  // 可以开始控制
    control_count = 0;      // 控制计数器清零 
    planner_trajectoryPub.publish(planner_trajectory);
    ROS_ERROR_STREAM("the planner_trajectory.traj size is " << planner_trajectory.traj.size());
}

void Planner::minimumJerkTrajGen(
    // Inputs:
    const int pieceNum,
    const Eigen::Vector3d &initialPos,
    const Eigen::Vector3d &initialVel,
    const Eigen::Vector3d &initialAcc,
    const Eigen::Vector3d &terminalPos,
    const Eigen::Vector3d &terminalVel,
    const Eigen::Vector3d &terminalAcc,
    const Eigen::Matrix3Xd &intermediatePositions,
    const Eigen::VectorXd &timeAllocationVector,
    // Outputs:
    Eigen::MatrixX3d &coefficientMatrix)
{
    /* x(t) = c0 + c1 * t + c2 * t^2 + c3 * t^3 + c4 * t^4 + c5 * t^5
    * coefficientMatrix = | c0_x c0_y c0_z |
    *                      | c0_x c0_y c0_z |
    *                      | c1_x c1_y c1_z |
    *                      | c2_x c2_y c2_z |
    *                      | c3_x c3_y c3_z |
    *                      | c4_x c4_y c4_z |
    *                      | c5_x c5_y c5_z |
    */

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(pieceNum * 6, pieceNum * 6);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(pieceNum * 6, 3);
    Eigen::Matrix<double, 3, 6> F_0, E_pieceNum;
    F_0 << 1, 0, 0, 0, 0, 0, 
           0, 1, 0, 0, 0, 0,
           0, 0, 2, 0, 0, 0;
    M.block(0, 0, 3, 6) = F_0;
    double T(timeAllocationVector(pieceNum - 1));
    E_pieceNum << 1 , T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5),
                  0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
                  0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
    M.block((pieceNum - 1) * 6 + 3, (pieceNum - 1) * 6, 3, 6) = E_pieceNum;
    b.block(0, 0, 3, 3) << initialPos(0), initialPos(1), initialPos(2),
                           initialVel(0), initialVel(1), initialVel(2),
                           initialAcc(0), initialAcc(1), initialAcc(2);
    b.block(3 + (pieceNum - 1) * 6, 0, 3, 3) << terminalPos(0), terminalPos(1), terminalPos(2),
                                                terminalVel(0), terminalVel(1), terminalVel(2),
                                                terminalAcc(0), terminalAcc(1), terminalAcc(2);
    Eigen::Matrix<double, 6, 6> F_i;
    F_i << 0, 0, 0, 0, 0, 0,
          -1, 0, 0, 0, 0, 0,
          0, -1, 0, 0, 0, 0,
          0, 0, -2, 0, 0, 0,
          0, 0, 0, -6, 0, 0,
          0, 0, 0, 0, -24, 0;
    for(int i = 1; i < pieceNum; ++i)
    {
        double t_i(timeAllocationVector(i - 1));
        Eigen::Matrix<double, 6, 6> E_i;
        Eigen::Vector3d D_i(intermediatePositions.transpose().row(i - 1));
        E_i << 1, t_i, pow(t_i, 2), pow(t_i, 3), pow(t_i, 4), pow(t_i, 5),
               1, t_i, pow(t_i, 2), pow(t_i, 3), pow(t_i, 4), pow(t_i, 5),
               0, 1, 2 * t_i, 3 * pow(t_i, 2), 4 * pow(t_i, 3), 5 * pow(t_i, 4),
               0, 0, 2, 6 * t_i, 12 * pow(t_i, 2), 20 * pow(t_i, 3),
               0, 0, 0, 6, 24 * t_i, 60 * pow(t_i, 2),
               0, 0, 0, 0, 24, 120 * t_i;
        M.block((i - 1) * 6 + 3, i * 6, 6, 6) = F_i;
        M.block((i - 1) * 6 + 3, (i - 1) * 6, 6, 6) = E_i;
        b.block((i - 1) * 6 + 3, 0, 6, 3) << D_i(0), D_i(1), D_i(2),
                                             0, 0, 0,
                                             0, 0, 0,
                                             0, 0, 0,
                                             0, 0, 0,
                                             0, 0, 0;
    }
    coefficientMatrix = M.partialPivLu().solve(b);
}

void Planner::TimeMatrix(double t, Eigen::Matrix<double, 3, 6> &time_matrix)
{
    time_matrix << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
                   0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
                   0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);
}

bool Planner::checkFirstPosition()
{
    if(std::fabs(vehicle_pose.position.x - first_position.position.x) > 0.1 ||
       std::fabs(vehicle_pose.position.y - first_position.position.y) > 0.1 ||
       std::fabs(vehicle_pose.position.z - first_position.position.z) > 0.1)
        return false;
    return true;
}

bool Planner::checkPosition(const Eigen::Vector3d p, const double accuracy)
{
    Eigen::Vector3d v_p(vehicle_pose.position.x, vehicle_pose.position.y, vehicle_pose.position.z);
    if((p - v_p).norm() < accuracy)
        return true;
    return false;
}


void Planner::MulticopterControl(const ros::TimerEvent&)
{
    if(!config.planner_trajectory.MultiCopterControl)
    {
        ROS_ERROR("Set the MultiCopterControl true!!!");
        ROS_BREAK();
    }
    if(home_position_sign)
    {
        std_msgs::Bool vision_sign;
        vision_sign.data = false;
        if(multiCopter_step < 2 + config.planner_trajectory.MinimumJerk_WayPoint.size() / 3)
        {
            if(0 == multiCopter_step)
            {
                ROS_INFO_STREAM("first_position");
                controlPub.publish(first_position);
                if(checkFirstPosition())
                    ++multiCopter_step;
            }
            else if(multiCopter_step == config.planner_trajectory.VisionControl && !multiCopter_vision_sign)    // 视觉控制
            {
                if(receive_vision_position_sign)      // 检查是否收到视觉定位的消息
                {
                    multicopter_task_position.position = vision_position.pose.position;
                    multicopter_task_position.position.z = first_position.position.z;
                    Eigen::Vector3d vision_p(vision_position.pose.position.x, vision_position.pose.position.y, vision_position.pose.position.z);
                    PidVelocityControl(vision_p, multicopter_task_position);
                    controlPub.publish(multicopter_task_position);
                    ROS_INFO_STREAM("control");
                    vision_positionPub.publish(vision_sign);
                    if(checkPosition(vision_p, 0.01))
                        multiCopter_vision_sign = true;
                }
                else        // 未收到则悬停
                {
                    vision_sign.data = true;
                    controlPub.publish(multicopter_task_position);
                    vision_positionPub.publish(vision_sign);
                    ROS_INFO_STREAM("sign");
                }
            }
            else
            {
                multicopter_task_position.position.x = internal_point.at(multiCopter_step).x();
                multicopter_task_position.position.y = internal_point.at(multiCopter_step).y();
                multicopter_task_position.position.z = internal_point.at(multiCopter_step).z();
                controlPub.publish(multicopter_task_position);
                if(checkPosition(internal_point.at(multiCopter_step), 0.05))
                    ++multiCopter_step;
            }
        }
        else
        {
            controlPub.publish(first_position);
        }
        // vision_positionPub.publish(vision_sign);
    }
}

void Planner::homePositionCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    home_position = *msg;
    home_position_sign = true;
}

void Planner::visionPositionCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    vision_position = *msg;
    receive_vision_position_sign = true;
}

void Planner::PidVelocityControl(const Eigen::Vector3d &p, mavros_msgs::PositionTarget &control)
{
    Eigen::Vector3d v_p(vehicle_pose.position.x, vehicle_pose.position.y, vehicle_pose.position.z);
    Eigen::Vector3d err_v = p - v_p;
    err_x = err_v.x(), err_y = err_v.y(), err_z = err_v.z();
    increase_vel_x = KP_X * (err_x - err_next_x) + KI_X * err_x + KD_X * (err_x - 2 * err_next_x + err_last_x);
	increase_vel_y = KP_Y * (err_y - err_next_y) + KI_Y * err_y + KD_Y * (err_y - 2 * err_next_y + err_last_y);
	increase_vel_z = KP_Z * (err_z - err_next_z) + KI_Z * err_z + KD_Z * (err_z - 2 * err_next_z + err_last_z);

    exp_vel.linear.x += increase_vel_x;
	exp_vel.linear.y += increase_vel_y;
	exp_vel.linear.z += increase_vel_z;

	err_last_x = err_next_x;
	err_next_x = err_x;

	err_last_y = err_next_y;
	err_next_y = err_y;

	err_last_z = err_next_z;
	err_next_z = err_z;
    control.velocity.x = exp_vel.linear.x;
    control.velocity.y = exp_vel.linear.y;
    control.velocity.z = exp_vel.linear.z;
}



} // namespace CUADC

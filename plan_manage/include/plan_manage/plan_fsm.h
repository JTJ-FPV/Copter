#pragma once

#include <ros/ros.h>
// 前端轨迹搜索
#include <grid_search/Astar_search.h>
// 后端轨迹优化
#include <trajectory_gen/minimumJerk.h>
// 相关参数
#include "fsmConfig.hpp"
#include <trajectory_msgs/Trajectory.h>

/* 多消息同步回调 */
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
// 轨迹优化所需的数据
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
// 控制接口
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
// 可视化
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace CUADC{

class PlanFSM
{
private:
    // FSM
    enum FSM_EXEC_STATE
    {
        TAKEOFF,
        INIT,       
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ
    };
    bool FSM[6]={false, false, false, false, false, false};

    TrajectoryGen trajectory_gen_, trajectory_replan_gen_;
    AstarPathFinder astar_finder_;/*, astar_replan_finder_;*/
    std::vector<Eigen::Vector3d> astar_path_, rdp_path_, astar_replan_path_, rdp_replan_path_;
    Map map;

    RePlanConfig config;
    
    /* ROS publisher */
    ros::NodeHandle nh_;
    // publisher
    ros::Publisher aster_path_pub_;
    ros::Publisher rdp_path_pub_;
    ros::Publisher jerk_traj_pub_;
    ros::Publisher control_pub_;
    // subscriber
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber target_sub_;
    typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::Imu> MySyncPolicy_odom_imu;
    
    // 机体位姿
    geometry_msgs::Pose vehicle_pose_; Eigen::Vector3d _vehicle_pose_, euler_angle_; Eigen::Quaterniond vehicle_q_;  bool has_odom_;
    // 规划的起始位置
    Eigen::Vector3d start_pt_; Eigen::Vector3i start_idx_;
    // 目标点
    Eigen::Vector3d target_pt_; Eigen::Vector3i target_idx_ ; bool has_target_;
    // 机体速度与加速度
    Eigen::Vector3d vel_, acc_;
    // 机器状态
    mavros_msgs::State state_;
    // 控制点
    std::vector<mavros_msgs::PositionTarget> minijerk_trajectory_;       // minijerk轨迹用于外部控制
    mavros_msgs::PositionTarget takeoff_position_;      // 起飞悬停点
    mavros_msgs::PositionTarget hover_position_; bool hover_sign_;    // 悬停点
    mavros_msgs::PositionTarget traj_poistion_;
    // 是否成功生成轨迹
    bool success_traj_gen_;
    // 轨迹开始时间
    ros::Time time_traj_start_; 
    // 轨迹总时间
    double traj_time_duration_;     uint32_t traj_time_count_, traj_piece_count_;
    // 轨迹时间参数矩阵
    Eigen::Matrix<double, 3, 6> traj_time_matrix_;
    // 轨迹可视化
    trajectory_msgs::Trajectory jerk_trajectory_;      bool jerk_trajectory_sign_, replan_jerk_trajectory_sign_;

    inline void changeState(FSM_EXEC_STATE new_state);

    void Init_paramters();
    void Init_publisher();
    void Init_subscriber();

    // void publish
    void odom_ImuCallback(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu);
    void stateCallback(const mavros_msgs::StateConstPtr &msg);
    void targetCallback(const geometry_msgs::PoseStampedConstPtr &goal);
    void visualCallback(const ros::TimerEvent &e);
    void execCallback(const ros::TimerEvent &e);
    void controlCallback(const ros::TimerEvent &e);

    // 可视化
    void publishAstarPath();
    void publishRdpPath();
    void publishTrajectory();

    void getPos(double t, TrajectoryGen &traj_gen, uint32_t piece, Eigen::Vector3d &pos);
    void getVel(double t, TrajectoryGen &traj_gen, uint32_t piece, Eigen::Vector3d &vel);
    void getAcc(double t, TrajectoryGen &traj_gen, uint32_t piece, Eigen::Vector3d &acc);
    
    bool trajGeneration(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc);
    bool ReplanTrajGeneration(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &vel, const Eigen::Vector3d &acc);
    bool safeCheck(TrajectoryGen &traj_gen, std::vector<Eigen::Vector3d> &path, Eigen::Vector3d &start_pt, Eigen::Vector3d &end_pt, uint32_t &index);
    bool replanSafeCheck();
    Eigen::Vector3d getMiddlePoint(const std::vector<Eigen::Vector3d> &path, const Eigen::Vector3d &start, const Eigen::Vector3d &end);

    ros::Time takeoff_last_rq_;

    ros::Timer _vis_timer;
    ros::Timer _exec_timer;
    ros::Timer _exec_control;
    FSM_EXEC_STATE exec_state_;
public:
    PlanFSM(ros::NodeHandle &nh):nh_(nh)
    {   
        // 读取相关参数
        config.getParamters(nh);
        // 地图
        map.initMap(nh);
        // 前端图搜索A*
        astar_finder_.initAstarPathFinder(&map);
        // astar_replan_finder_.initAstarPathFinder(&map);
        // 后端轨迹优化器MinimumJerk
        trajectory_gen_.initTrajectoryGen(config.copter.max_acc, config.copter.max_vel);
        trajectory_replan_gen_.initTrajectoryGen(config.copter.max_acc, config.copter.max_vel);
        // 初始化订阅
        Init_subscriber();
        // 初始化发布
        Init_publisher();
        // 初始化相关参数
        Init_paramters();
    };
    ~PlanFSM(){};
};

} // namespace CUADC

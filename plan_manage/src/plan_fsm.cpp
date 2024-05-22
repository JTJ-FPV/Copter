#include "plan_manage/plan_fsm.h"

namespace CUADC{

inline void PlanFSM::changeState(FSM_EXEC_STATE new_state)
{
    std::string state_str[6] = {"TAKEOFF", "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    ROS_INFO_STREAM("[STATE]: from " << state_str[pre_s] << " to " << state_str[int(new_state)]);
}

void PlanFSM::Init_paramters()
{
    // 初始化状态机
    exec_state_ = FSM_EXEC_STATE::TAKEOFF;
    has_target_ = false;
    has_odom_ = false;
    hover_sign_ = false;
    success_traj_gen_ = false;
    jerk_trajectory_sign_ = false;
    replan_jerk_trajectory_sign_ = false;
    traj_time_count_ = 0;
    traj_piece_count_ = 0;
    yaw_ = 0;
    
    takeoff_position_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    takeoff_position_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    hover_position_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    hover_position_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    traj_poistion_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    traj_poistion_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    jerk_trajectory_.Header.frame_id = map.mp_.frame_id_;

}

void PlanFSM::Init_publisher()
{
    aster_path_pub_ = nh_.advertise<visualization_msgs::Marker>(config.publish_replan.Publish_AstarPath, 1);
    replan_aster_path_pub_ = nh_.advertise<visualization_msgs::Marker>(config.publish_replan.Publish_Replan_AstarPath, 1);
    rdp_path_pub_ = nh_.advertise<visualization_msgs::Marker>(config.publish_replan.Publish_RdpPath, 1);
    jerk_traj_pub_ = nh_.advertise<trajectory_msgs::Trajectory>(config.publish_replan.Publish_MinimJerkPath, 1);
    control_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(config.publish_replan.Publish_Control, 1);
    _vis_timer = nh_.createTimer(ros::Duration(0.1), &PlanFSM::visualCallback, this);
    _exec_timer = nh_.createTimer(ros::Duration(0.01), &PlanFSM::execCallback, this);
    _exec_control = nh_.createTimer(ros::Duration(ros::Rate(20)), &PlanFSM::controlCallback, this);
}

void PlanFSM::Init_subscriber()
{
    static message_filters::Subscriber<nav_msgs::Odometry> odom_data(nh_, config.subscribe_replan.Subscribe_Odometry, 1, ros::TransportHints().tcpNoDelay());
    static message_filters::Subscriber<sensor_msgs::Imu> imu_data(nh_, config.subscribe_replan.Subscribe_Imu, 1, ros::TransportHints().tcpNoDelay());
    static message_filters::Synchronizer<MySyncPolicy_odom_imu> sync_odom_imu(MySyncPolicy_odom_imu(1), odom_data, imu_data);
    sync_odom_imu.registerCallback(boost::bind(&PlanFSM::odom_ImuCallback, this, _1, _2));
    state_sub_ = nh_.subscribe<mavros_msgs::State>(config.subscribe_replan.Subscribe_State, 10, &PlanFSM::stateCallback, this, ros::TransportHints().tcpNoDelay());
    target_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/goal", 10, &PlanFSM::targetCallback, this, ros::TransportHints().tcpNoDelay());
    
}

void PlanFSM::odom_ImuCallback(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImuConstPtr &imu)
{
    geometry_msgs::Pose pose = odom->pose.pose;         this->vehicle_pose_ = pose;
    geometry_msgs::Twist velocity_b = odom->twist.twist;
    Eigen::Vector3d /*P_w(pose.position.x ,pose.position.y, pose.position.z),*/ V_b(velocity_b.linear.x, velocity_b.linear.y, velocity_b.linear.z);
    Eigen::Quaterniond q_wb(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    
    _vehicle_pose_ << pose.position.x, pose.position.y, pose.position.z;

    vehicle_q_ = q_wb;

    euler_angle_ = q_wb.toRotationMatrix().eulerAngles(2, 1, 0);
    
    // euler_angle_(0) = euler_angle_(0) + M_PI; 

    vel_ = q_wb * V_b;   // 将速度转换成世界坐标系下的速度
    /* 获取加速度数据 */
    Eigen::Vector3d acc_b(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    acc_ = q_wb * acc_b;   // 将机体坐标下的加速度转换成世界坐标系下的加速度
    has_odom_ = true;
}

void PlanFSM::stateCallback(const mavros_msgs::StateConstPtr &msg)
{
    state_ = *msg;
}

void PlanFSM::targetCallback(const geometry_msgs::PoseStampedConstPtr &goal)
{   
    if(exec_state_ != FSM_EXEC_STATE::WAIT_TARGET)
    {
        ROS_WARN("target has not been reached yet!!");
        return;
    }
    target_pt_ << goal->pose.position.x, goal->pose.position.y, goal->pose.position.z;
    start_pt_ << vehicle_pose_.position.x, vehicle_pose_.position.y, vehicle_pose_.position.z;
    if(!map.isInMap(target_pt_))
    {
        ROS_WARN("the target is not in map!");
        has_target_ = false;
        return;
    }
    map.pos2Index(start_pt_, start_idx_);
    map.pos2Index(target_pt_, target_idx_);
    if(map.isOccupied(target_idx_) == VoxelState::OCCUPANY)
    {
        ROS_WARN("the target is occupancied!");
        has_target_ = false;
    }
    else
        has_target_ = true;
}

bool PlanFSM::trajGeneration(const Eigen::Vector3d &vel, const Eigen::Vector3d &acc)
{
    astar_path_.clear();
    rdp_path_.clear();
    // 控制
    traj_piece_count_ = 0;
    traj_time_count_ = 0;
    // 前端 A* 搜索
    astar_finder_.AstarGraphSearch(start_pt_, target_pt_, AstarHeu::DIALOG);
    // 获取路径
    astar_finder_.getPath(astar_path_);
    if(!astar_path_.size())
    {
        ROS_INFO_STREAM("the A* path size is " << astar_path_.size());
        ROS_WARN("A* search path failed!");
        jerk_trajectory_sign_ = false; // 轨迹可视化
        return false;
    }

    // 轨迹简化
    astar_finder_.simplifyPath(astar_path_, rdp_path_);
    ROS_INFO_STREAM("simplified path size is " << rdp_path_.size());
    if(!rdp_path_.size())
    {
        ROS_WARN("simplified path failed using rdp simplified ...");
        astar_finder_.rdpPath(astar_path_, 0.3, rdp_path_);
    }

    if(!rdp_path_.size())
    {
        ROS_WARN("rdp simplified path failed!");
        jerk_trajectory_sign_ = false; // 轨迹可视化
        return false;
    }

    Eigen::MatrixXd positions = Eigen::MatrixXd::Zero(3, rdp_path_.size());
    for(uint32_t i = 0; i < rdp_path_.size(); ++i)
    {
        positions.block<3, 1>(0, i) = rdp_path_.at(i);
    }
    // 后端轨迹优化
    if(!trajectory_gen_.initTraj(positions, vel, acc))
    {
        ROS_INFO("trajectory optimization initialization failed!");
        jerk_trajectory_sign_ = false; // 轨迹可视化
        return false;
    }
    trajectory_gen_.minimumJerkTrajGen();

    // 安全检查
    Eigen::Vector3d start_pt, end_pt;
    // Eigen::Vector3i start_id, end_id;
    uint32_t index;
    bool safe_sign = safeCheck(trajectory_gen_, rdp_path_, start_pt, end_pt, index);

    // 若无碰撞则退出
    if(safe_sign)
    {
        ROS_INFO("No Collision!");
        jerk_trajectory_sign_ = true; // 轨迹可视化
        time_traj_start_ = ros::Time::now();
        traj_time_duration_ = trajectory_gen_.TotalTime();
        return true;
    }

    ROS_INFO("Check Collision!");
    // 若发生碰撞则在rdp之间插点
    Eigen::Vector3d middle_pt, last_middle_pt;
    bool middle_sign = true;

    // std::vector<Eigen::Vector3d> safe_check_insert, safe_check_insert_simplify;
    
    ros::Time safe_check_start = ros::Time::now();
    while(!safe_sign)
    {
        // safe_check_insert.clear();
        // safe_check_insert_simplify.clear();
        // 是否超时
        if(ros::Time::now() - safe_check_start > ros::Duration(0.05))
        {
            ROS_WARN("safe reoptimization time is over 50ms!");
            jerk_trajectory_sign_ = false; // 轨迹可视化
            return false;
        }

        // // 用于检测起点重点是否在地图中重合
        // map.pos2Index(start_pt, start_id);
        // map.pos2Index(end_pt, end_id);
        // if(start_id == end_id)
        // {
        //     ROS_WARN("insert point between the obstacle point but still cannot finding a safe trajectory!");
        //     return false;
        // }
        //  // 进行图搜索
        // astar_finder_.AstarGraphSearch(start_pt, end_pt, AstarHeu::DIALOG);
        // // 获得路径
        // astar_finder_.getPath(safe_check_insert);
        // if(!safe_check_insert.size())
        // {
        //     ROS_WARN("safe check intend to search a path by A* but failed!");
        //     return false;
        // }
        // // 简化路径
        // astar_finder_.simplifyPath(safe_check_insert, safe_check_insert_simplify);
        // if(!safe_check_insert_simplify.size())
        // {
        //     ROS_WARN("safe check simplify path failed!");
        //     return false;
        // }
        // // 将简化的路径插入到rdp_path
        // index++;
        // if(index >= 0 && index <= rdp_path_.size()){
        //     rdp_path_.insert(rdp_path_.begin() + index, safe_check_insert_simplify.begin(), safe_check_insert_simplify.end());
        // }
        // else{
        //     ROS_WARN("safe check insert the search path failed!");
        //     return false;
        // }

   
        // 若发生碰撞则在rdp之间插点
        if(middle_sign)
        {
            // middle_pt = getMiddlePoint(astar_path_, start_pt, end_pt);
            middle_pt = (start_pt + end_pt) / 2;
            last_middle_pt = middle_pt;
            middle_sign = false;
        }
        else
        {
            last_middle_pt = middle_pt;
            // middle_pt = getMiddlePoint(astar_path_, start_pt, end_pt);
            middle_pt = (start_pt + end_pt) / 2;
            // map.pos2Index(last_middle_pt, last_middle_idx);
            // map.pos2Index(middle_pt, middle_idx);
            // if(middle_idx == last_middle_idx)
            // {
            //     ROS_WARN("insert middle point failed!");
            //     jerk_trajectory_sign_ = false; // 轨迹可视化
            //     return false;
            // }
        }
        rdp_path_.emplace(rdp_path_.begin() + index + 1, middle_pt);


        positions = Eigen::MatrixXd::Zero(3, rdp_path_.size());
        for(uint32_t i = 0; i < rdp_path_.size(); ++i)
        {   
            positions.block<3, 1>(0, i) = rdp_path_.at(i);
        }
        // ROS_INFO_STREAM("the rdp path size is " << rdp_path_.size());
        // 后端轨迹优化
        if(!trajectory_gen_.initTraj(positions, vel, acc))
        {
            ROS_INFO("trajectory optimization initialization failed!");
            jerk_trajectory_sign_ = false; // 轨迹可视化
            return false;
        }
        // 开始优化
        if(!trajectory_gen_.minimumJerkTrajGen())
        {
            ROS_WARN("Optimization failed!");
            replan_jerk_trajectory_sign_ = false; // 轨迹可视化
            return false;
        }

        // 继续安全检查
        safe_sign = safeCheck(trajectory_gen_, rdp_path_, start_pt, end_pt, index);
    }
    traj_time_duration_ = trajectory_gen_.TotalTime();
    time_traj_start_ = ros::Time::now();
    jerk_trajectory_sign_ = true; // 轨迹可视化
    return true;
}

bool PlanFSM::ReplanTrajGeneration(const Eigen::Vector3d &future_pt, const Eigen::Vector3d &vel, const Eigen::Vector3d &acc)
{
    astar_replan_path_.clear();
    rdp_replan_path_.clear();
    if(isnan(future_pt.x()) || isnan(future_pt.y()) || isnan(future_pt.z()))
    {
        ROS_WARN("the replan A* start point is illegal!");
        replan_jerk_trajectory_sign_ = false; // 轨迹可视化
        return false;
    }
    if(!map.isInMap(future_pt))
    {
        ROS_WARN("the replan A* start point is not in map!");
        replan_jerk_trajectory_sign_ = false; // 轨迹可视化
        return false;
    }
    Eigen::Vector3i future_idx;
    map.pos2Index(future_pt, future_idx);
    if(VoxelState::OCCUPANY == map.isOccupied(future_idx))
    {
        ROS_WARN("the replan A* start point is occupancyied!");
        replan_jerk_trajectory_sign_ = false; // 轨迹可视化
        return false;
    }
    // 前端 A* 搜索
    astar_finder_.AstarGraphSearch(future_pt, target_pt_, AstarHeu::DIALOG);
    // 获取路径
    astar_finder_.getPath(astar_replan_path_);
    if(!astar_replan_path_.size())
    {
        ROS_INFO_STREAM("the replan A* path size is " << astar_replan_path_.size());
        ROS_WARN("replan A* search path failed!");
        replan_jerk_trajectory_sign_ = false; // 轨迹可视化
        return false;
    }

    // 轨迹简化
    astar_finder_.simplifyPath(astar_replan_path_, rdp_replan_path_);
    ROS_INFO_STREAM("replan simplified path size is " << rdp_replan_path_.size());
    if(!rdp_replan_path_.size())
    {
        ROS_WARN("replan simplified path failed using rdp simplified ...");
        astar_finder_.rdpPath(astar_replan_path_, 0.15, rdp_replan_path_);
    }

    if(!rdp_replan_path_.size())
    {
        ROS_WARN("replan rdp simplified path failed!");
        replan_jerk_trajectory_sign_ = false; // 轨迹可视化
        return false;
    }

    Eigen::MatrixXd positions = Eigen::MatrixXd::Zero(3, rdp_replan_path_.size());
    for(uint32_t i = 0; i < rdp_replan_path_.size(); ++i)
    {
        positions.block<3, 1>(0, i) = rdp_replan_path_.at(i);
    }
    // 后端轨迹优化
    if(!trajectory_replan_gen_.initTraj(positions, vel, acc))
    {
        ROS_INFO("replan trajectory optimization initial failed!");
        replan_jerk_trajectory_sign_ = false; // 轨迹可视化
        return false;
    }
    // 开始优化
    ROS_INFO("replan begin optimization");
    if(!trajectory_replan_gen_.minimumJerkTrajGen())
    {
        ROS_WARN("Optimization failed!");
        replan_jerk_trajectory_sign_ = false; // 轨迹可视化
        return false;
    }

    // 安全检查
    Eigen::Vector3d start_pt, end_pt;
    // Eigen::Vector3i start_id, end_id;
    uint32_t index;
    bool safe_sign = safeCheck(trajectory_replan_gen_, rdp_replan_path_, start_pt, end_pt, index);

    // 若无碰撞则退出
    if(safe_sign)
    {
        ROS_INFO("No Collision!");
        // 控制
        traj_piece_count_ = 0;
        traj_time_count_ = 0;
        replan_jerk_trajectory_sign_ = true; // 轨迹可视化
        time_traj_start_ = ros::Time::now();
        traj_time_duration_ = trajectory_gen_.TotalTime();
        return true;
    }

    ROS_INFO("Check Collision!");
    // 若发生碰撞则在rdp之间插点
    Eigen::Vector3d middle_pt, last_middle_pt;
    bool middle_sign = true;

    // std::vector<Eigen::Vector3d> safe_check_insert, safe_check_insert_simplify;

    ros::Time safe_check_start = ros::Time::now();
    while(!safe_sign)
    {
        // safe_check_insert.clear();
        // safe_check_insert_simplify.clear();
        // 是否超时
        if(ros::Time::now() - safe_check_start > ros::Duration(0.05))
        {
            ROS_WARN("replan safe optimization time is over 50ms!");
            replan_jerk_trajectory_sign_ = false; // 轨迹可视化
            return false;
        }

        // 用于检测起点重点是否在地图中重合
        // map.pos2Index(start_pt, start_id);
        // map.pos2Index(end_pt, end_id);
        // if(start_id == end_id)
        // {
        //     ROS_WARN("replan insert point between the obstacle point but still cannot finding a safe trajectory!");
        //     return false;
        // }
        // // 进行图搜索
        // astar_finder_.AstarGraphSearch(start_pt, end_pt, AstarHeu::DIALOG);
        // // 获得路径
        // astar_finder_.getPath(safe_check_insert);
        // if(!safe_check_insert.size())
        // {
        //     ROS_WARN("replan safe check intend to search a path by A* but failed!");
        //     return false;
        // }
        // // 简化路径
        // astar_finder_.simplifyPath(safe_check_insert, safe_check_insert_simplify);
        // if(!safe_check_insert_simplify.size())
        // {
        //     ROS_WARN("replan safe check simplify path failed!");
        //     return false;
        // }
        // // 将简化的路径插入到replan_rdp_path
        // index++;
        // if(index >= 0 && index <= rdp_replan_path_.size()){
        //     rdp_replan_path_.insert(rdp_replan_path_.begin() + index, safe_check_insert_simplify.begin(), safe_check_insert_simplify.end());
        // }
        // else{
        //     ROS_WARN("replan safe check insert the search path failed!");
        //     return false;
        // }

        
        // 若发生碰撞则在rdp之间插点
        if(middle_sign)
        {
            // middle_pt = getMiddlePoint(astar_replan_path_, start_pt, end_pt);
            middle_pt = (start_pt + end_pt) / 2;
            last_middle_pt = middle_pt;
            middle_sign = false;
        }
        else
        {
            last_middle_pt = middle_pt;
            // middle_pt = getMiddlePoint(astar_replan_path_, start_pt, end_pt);
            middle_pt = (start_pt + end_pt) / 2;
            // map.pos2Index(last_middle_pt, last_middle_idx);
            // map.pos2Index(middle_pt, middle_idx);
            // if(middle_idx == last_middle_idx)
            // {
            //     ROS_WARN("insert middle point failed!");
            //     jerk_trajectory_sign_ = false; // 轨迹可视化
            //     return false;
            // }
        }
        rdp_replan_path_.emplace(rdp_replan_path_.begin() + index + 1, middle_pt);
       
       
        positions = Eigen::MatrixXd::Zero(3, rdp_replan_path_.size());
        for(uint32_t i = 0; i < rdp_replan_path_.size(); ++i)
        {   
            positions.block<3, 1>(0, i) = rdp_replan_path_.at(i);
        }

        // 后端轨迹优化
        if(!trajectory_replan_gen_.initTraj(positions, vel, acc))
        {
            ROS_WARN("trajectory optimization initialization failed!");
            replan_jerk_trajectory_sign_ = false; // 轨迹可视化
            return false;
        }
        // 开始优化
        ROS_INFO("replan begin optimization");

        // 若未未优化则返回false
        if(!trajectory_replan_gen_.minimumJerkTrajGen())
        {
            ROS_WARN("Optimization failed!");
            replan_jerk_trajectory_sign_ = false; // 轨迹可视化
            return false;
        }

        // 继续安全检查
        safe_sign = safeCheck(trajectory_replan_gen_, rdp_replan_path_, start_pt, end_pt, index);

    }
    // 控制
    traj_piece_count_ = 0;
    traj_time_count_ = 0;
    traj_time_duration_ = trajectory_replan_gen_.TotalTime();
    time_traj_start_ = ros::Time::now();
    replan_jerk_trajectory_sign_ = true; // 轨迹可视化
    return true;
}

bool PlanFSM::replanSafeCheck()
{
    double dt = config.replan.dt;
    Eigen::Vector3d pos;
    Eigen::Vector3i pos_idx;
    uint32_t count = 0;
    for(uint32_t i = traj_piece_count_; i < trajectory_gen_.getPieceNum(); ++i){
        if(i != traj_piece_count_)
            count = 0;
        else
            count = traj_time_count_;
        while(count * dt <= trajectory_gen_.getPieceTime(i))
        {
            getPos(count++ * dt, trajectory_gen_, i, pos);
            if(!map.isInMap(pos))
                return false;   
            map.pos2Index(pos, pos_idx);
            if(VoxelState::OCCUPANY == map.isOccupied(pos_idx))
                return false;
        }
    }
    return true;
}

bool PlanFSM::safeCheck(TrajectoryGen &traj_gen, std::vector<Eigen::Vector3d> &path, Eigen::Vector3d &start_pt, Eigen::Vector3d &end_pt, uint32_t &index)
{

    double dt = config.replan.dt;
    Eigen::Matrix3d traj_state;
    Eigen::Vector3d point_pt;
    Eigen::Vector3i point_idx;
    for(uint32_t i = 0; i < traj_gen.getPieceNum(); ++i)
    {
        uint32_t count = 0;
        while(count * dt <= traj_gen.getPieceTime(i))
        {
            traj_gen.TimeMatrix(count * dt, traj_time_matrix_);
            traj_state = traj_time_matrix_ * traj_gen.coefficientMatrix_.block<6, 3>(6 * i, 0);
            point_pt << traj_state(0, 0), traj_state(0, 1), traj_state(0, 2);
            // 是否在地图内
            if(!map.isInMap(point_pt))
            {
                start_pt << path.at(i).x(), path.at(i).y(), path.at(i).z();
                end_pt << path.at(i + 1).x(), path.at(i + 1).y(), path.at(i + 1).z();
                index = i;
                return false;
            }
            // 是否为障碍物
            map.pos2Index(point_pt, point_idx);
            if(VoxelState::OCCUPANY == map.isOccupied(point_idx))
            {
                start_pt << path.at(i).x(), path.at(i).y(), path.at(i).z();
                end_pt << path.at(i + 1).x(), path.at(i + 1).y(), path.at(i + 1).z();
                index = i;
                return false;
            }
            count++;
        }
    }
    return true;
}

Eigen::Vector3d PlanFSM::getMiddlePoint(const std::vector<Eigen::Vector3d> &path, const Eigen::Vector3d &start, const Eigen::Vector3d &end)
{
    uint32_t start_id = 0, end_id = 0;
    for(uint32_t i = 0; i < path.size(); ++i)
    {
        if(path.at(i) == start)
            start_id = i;
        else if(path.at(i) == end)
        {
            end_id = i;
            break;
        }
    }
    return path.at((start_id + end_id) / 2);
}

void PlanFSM::getPos(double t, TrajectoryGen &traj_gen, uint32_t piece, Eigen::Vector3d &pos)
{
    Eigen::Matrix<double, 3, 6> time_matrix;
    traj_gen.TimeMatrix(t, time_matrix);
    pos = (time_matrix * traj_gen.coefficientMatrix_.block<6, 3>(6 * piece, 0)).block<1, 3>(0, 0).transpose();
}

void PlanFSM::getVel(double t, TrajectoryGen &traj_gen, uint32_t piece, Eigen::Vector3d &vel)
{
    Eigen::Matrix<double, 3, 6> time_matrix;
    traj_gen.TimeMatrix(t, time_matrix);
    vel = (time_matrix * traj_gen.coefficientMatrix_.block<6, 3>(6 * piece, 0)).block<1, 3>(1, 0).transpose();
}

void PlanFSM::getAcc(double t, TrajectoryGen &traj_gen, uint32_t piece, Eigen::Vector3d &acc)
{
    Eigen::Matrix<double, 3, 6> time_matrix;
    traj_gen.TimeMatrix(t, time_matrix);
    acc = (time_matrix * traj_gen.coefficientMatrix_.block<6, 3>(6 * piece, 0)).block<1, 3>(2, 0).transpose();
}

void PlanFSM::publishAstarPath()
{
    if(aster_path_pub_.getNumSubscribers() <= 0)
        return;
    visualization_msgs::Marker node_vis_a; 
    node_vis_a.header.frame_id = map.mp_.frame_id_;
    node_vis_a.header.stamp = ros::Time::now();

    node_vis_a.type = visualization_msgs::Marker::CUBE_LIST;  // 立方体
    node_vis_a.action = visualization_msgs::Marker::ADD;
    node_vis_a.id = 1;

    node_vis_a.pose.orientation.x = 0.0;
    node_vis_a.pose.orientation.y = 0.0;
    node_vis_a.pose.orientation.z = 0.0;
    node_vis_a.pose.orientation.w = 1.0;

    node_vis_a.color.a = 1.0;
    node_vis_a.color.r = 1.0;
    node_vis_a.color.g = 1.0;
    node_vis_a.color.b = 1.0;

    node_vis_a.scale.x = map.mp_.resolution_;
    node_vis_a.scale.y = map.mp_.resolution_;
    node_vis_a.scale.z = map.mp_.resolution_;
    
    geometry_msgs::Point pt;
    for(auto &p:astar_path_)
    {
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();

        node_vis_a.points.push_back(pt);
    }
    aster_path_pub_.publish(node_vis_a);
}

void PlanFSM::publishReplanAstarPath()
{
    if(replan_aster_path_pub_.getNumSubscribers() <= 0)
        return;
    visualization_msgs::Marker node_vis_a; 
    node_vis_a.header.frame_id = map.mp_.frame_id_;
    node_vis_a.header.stamp = ros::Time::now();

    node_vis_a.type = visualization_msgs::Marker::CUBE_LIST;  // 立方体
    node_vis_a.action = visualization_msgs::Marker::ADD;
    node_vis_a.id = 2;

    node_vis_a.pose.orientation.x = 0.0;
    node_vis_a.pose.orientation.y = 0.0;
    node_vis_a.pose.orientation.z = 0.0;
    node_vis_a.pose.orientation.w = 1.0;

    node_vis_a.color.a = 1.0;
    node_vis_a.color.r = 0.0;
    node_vis_a.color.g = 1.0;
    node_vis_a.color.b = 1.0;

    node_vis_a.scale.x = map.mp_.resolution_;
    node_vis_a.scale.y = map.mp_.resolution_;
    node_vis_a.scale.z = map.mp_.resolution_;
    
    geometry_msgs::Point pt;
    for(auto &p:astar_replan_path_)
    {
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();

        node_vis_a.points.push_back(pt);
    }
    replan_aster_path_pub_.publish(node_vis_a);
}

void PlanFSM::publishRdpPath()
{
    if(rdp_path_pub_.getNumSubscribers() <= 0)
        return;
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = map.mp_.frame_id_;
    node_vis.header.stamp = ros::Time::now();

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;  // 立方体
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 0.8;
    node_vis.color.g = 0.3;
    node_vis.color.b = 0.8;

    node_vis.scale.x = map.mp_.resolution_;
    node_vis.scale.y = map.mp_.resolution_;
    node_vis.scale.z = map.mp_.resolution_;

    geometry_msgs::Point pt;
    for(auto &p:rdp_path_)
    {
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();

        node_vis.points.push_back(pt);
    }
    rdp_path_pub_.publish(node_vis);
}

void PlanFSM::publishTrajectory()
{
    if(jerk_traj_pub_.getNumSubscribers() <= 0  || !success_traj_gen_)
    {
        jerk_trajectory_sign_ = false;
        return;
    }

    if(jerk_trajectory_sign_)
    {
        jerk_trajectory_.traj.clear();
        jerk_trajectory_.Header.stamp = ros::Time::now();
        Eigen::Matrix3d traj_state;
        double dt = 0.01;
        geometry_msgs::PoseStamped traj_point;
        traj_point.header.frame_id = map.mp_.frame_id_;
        for(uint32_t i = 0; i < trajectory_gen_.getPieceNum(); ++i)
        {
            uint32_t count = 0;
            while(count * dt <= trajectory_gen_.getPieceTime(i))
            {
                trajectory_gen_.TimeMatrix((count++) * dt, traj_time_matrix_);
                traj_state = traj_time_matrix_ * trajectory_gen_.coefficientMatrix_.block<6, 3>(6 * i, 0);
                /* 轨迹离散化 */
                traj_point.pose.position.x = traj_state(0, 0);
                traj_point.pose.position.y = traj_state(0, 1);
                traj_point.pose.position.z = traj_state(0, 2);
                jerk_trajectory_.traj.push_back(traj_point);
            }
        }
        jerk_traj_pub_.publish(jerk_trajectory_);
    }
    else if(replan_jerk_trajectory_sign_)
    {
        jerk_trajectory_.traj.clear();
        jerk_trajectory_.Header.stamp = ros::Time::now();
        Eigen::Matrix3d traj_state;
        double dt = 0.01;
        geometry_msgs::PoseStamped traj_point;
        traj_point.header.frame_id = map.mp_.frame_id_;
        for(uint32_t i = 0; i < trajectory_replan_gen_.getPieceNum(); ++i)
        {
            uint32_t count = 0;
            while(count * dt <= trajectory_replan_gen_.getPieceTime(i))
            {
                trajectory_replan_gen_.TimeMatrix((count++) * dt, traj_time_matrix_);
                traj_state = traj_time_matrix_ * trajectory_replan_gen_.coefficientMatrix_.block<6, 3>(6 * i, 0);
                /* 轨迹离散化 */
                traj_point.pose.position.x = traj_state(0, 0);
                traj_point.pose.position.y = traj_state(0, 1);
                traj_point.pose.position.z = traj_state(0, 2);
                jerk_trajectory_.traj.push_back(traj_point);
            }
        }
        jerk_traj_pub_.publish(jerk_trajectory_);
    }
}

// 可视化
void PlanFSM::visualCallback(const ros::TimerEvent &e)
{
    publishAstarPath();
    publishReplanAstarPath();
    publishRdpPath();
    publishTrajectory();
}

// 状态机执行100hz
void PlanFSM::execCallback(const ros::TimerEvent &e)
{
    switch (exec_state_)
    {
        case FSM_EXEC_STATE::TAKEOFF:
        {
            // ROS_INFO("TAKEOFF");
            if(!has_odom_)
                return;
            // if(!state_.connected || !state_.armed || "OFFBOARD" == state_.mode)
            //     return;
            // takeoff_last_rq_ = ros::Time::now();
            if (std::fabs(vehicle_pose_.position.z - config.copter.hover_height) < 0.05 /*&& (ros::Time::now() - takeoff_last_rq_) > ros::Duration(5.0)*/)
            {
                changeState(FSM_EXEC_STATE::INIT);
            }
            FSM[(uint16_t)TAKEOFF] = true;
            break;
        }
        case FSM_EXEC_STATE::INIT:
        {
            // ROS_INFO("INIT");
            if(!has_odom_)
                return;
            changeState(FSM_EXEC_STATE::WAIT_TARGET);
            // FSM[(uint16_t)exec_state_] = true;
            break;
        }
        case FSM_EXEC_STATE::WAIT_TARGET:
        {
            // ROS_INFO("WAIT_TARGET");
            if(!has_target_)    // 悬停
            {
                if(!hover_sign_)
                {
                    hover_position_.position = vehicle_pose_.position;
                    hover_position_.yaw = yaw_;
                    hover_sign_ = true;
                    FSM[(uint16_t)exec_state_] = true;
                }
                control_pub_.publish(hover_position_);
                return;   
            }
            else    // 收到目标点
            {
                changeState(FSM_EXEC_STATE::GEN_NEW_TRAJ);
                hover_sign_ = false;
                FSM[(uint16_t)exec_state_] = false;
                return;
            }
            break;
        }
        case FSM_EXEC_STATE::GEN_NEW_TRAJ:
        {
            ROS_INFO("GEN_NEW_TRAJ");
            success_traj_gen_ = trajGeneration(vel_, acc_);
            if(success_traj_gen_)
            {
                changeState(EXEC_TRAJ);
                return;
            }
            else
            {
                has_target_ = false;
                hover_sign_ = false;
                changeState(WAIT_TARGET);
                return;
            }
            break;
        }
        case FSM_EXEC_STATE::REPLAN_TRAJ:
        {
            ROS_INFO("REPLAN_TRAJ");
            double current = traj_time_count_ * config.replan.dt;
            uint32_t piece = traj_piece_count_;
            if(piece >= trajectory_gen_.getPieceNum())
                piece = trajectory_gen_.getPieceNum() - 1;
            if(current + 0.02 > trajectory_gen_.getPieceTime(piece))
            {
                current = trajectory_gen_.getPieceTime(piece);
            }
            else
                current += 0.02;
            Eigen::Vector3d pos, vel, acc;
            Eigen::Vector3i pos_idx;
            getPos(current, trajectory_gen_, piece, pos);
            // map.pos2Index(pos, pos_idx);
            // if(VoxelState::OCCUPANY == map.isOccupied(pos_idx))
            // {
            //     jerk_trajectory_sign_ = false;
            //     replan_jerk_trajectory_sign_ = false;
            //     has_target_ = false;
            //     Eigen::Vector3d pos_last, d;
            //     getPos(current - 0.02 - config.replan.dt, trajectory_gen_, piece, pos_last);
            //     d = pos - pos_last;
            //     hover_position_.position.x = pos.x();
            //     hover_position_.position.y = pos.y();
            //     hover_position_.position.z = pos.z();
            //     hover_position_.yaw = atan2(d.y(), d.x());
            //     hover_sign_ = true;
            //     start_pt_ << vehicle_pose_.position.x, vehicle_pose_.position.y, vehicle_pose_.position.z;
            //     ROS_INFO("Future pos is occupancied");
            //     changeState(WAIT_TARGET);
            //     return;
            // }
            getVel(current, trajectory_gen_, piece, vel);
            getAcc(current, trajectory_gen_, piece, acc);
            
            success_traj_gen_ = ReplanTrajGeneration(pos, vel, acc);
            
            if(!success_traj_gen_)
            {
                ROS_WARN("No tajectory Optimaztion!");
                ROS_INFO_STREAM("success_traj_gen_" << success_traj_gen_);
                replan_jerk_trajectory_sign_ = false;
                changeState(GEN_NEW_TRAJ);
                return;
            }
            else if(success_traj_gen_)
            {
                start_pt_ = pos;
                trajectory_gen_ = trajectory_replan_gen_;
                // 控制
                traj_piece_count_ = 0;
                traj_time_count_ = 0;
                changeState(EXEC_TRAJ);
                return;
            }
            break;
        }
        case FSM_EXEC_STATE::EXEC_TRAJ:
        {
            // ROS_INFO_STREAM("distance from vehile pose to target is " << (_vehicle_pose_ - target_pt_).norm());
            if((_vehicle_pose_ - target_pt_).norm() <= 0.2){
                ROS_INFO_STREAM("distance from vehile pose to target is " << (_vehicle_pose_ - target_pt_).norm());
                ROS_INFO("REACH THE TARGET");
                has_target_ = false;
                hover_sign_ = false;
                changeState(FSM_EXEC_STATE::WAIT_TARGET);
                FSM[(uint16_t)exec_state_] = false;
                return;
            } else if(map.mp_.occ_need_update_){// 地图更新
                
                if(map.isOccupied(target_idx_) == VoxelState::OCCUPANY)
                {
                    ROS_WARN("the target is occupancied!");
                    has_target_ = false;

                    double current = traj_time_count_ * config.replan.dt;
                    uint32_t piece = traj_piece_count_;
                    if(piece >= trajectory_gen_.getPieceNum())
                        piece = trajectory_gen_.getPieceNum() - 1;
                    if(current + 0.1 > trajectory_gen_.getPieceTime(piece))
                    {
                        current = trajectory_gen_.getPieceTime(piece);
                    }
                    else
                        current += 0.1;
                        
                    Eigen::Vector3d pos, pos_future, d;
                    getPos(current - 0.1, trajectory_gen_, piece, pos);
                    getPos(current, trajectory_gen_, piece, pos_future);
                    d = pos_future - pos;
                    hover_position_.position.x = pos_future.x();
                    hover_position_.position.y = pos_future.y();
                    hover_position_.position.z = pos_future.z();
                    hover_position_.yaw = atan2(d.y(), d.x());
                    // hover_position_.position = vehicle_pose_.position;
                    // hover_position_.yaw = yaw_;
                    hover_sign_ = true;
                    changeState(FSM_EXEC_STATE::WAIT_TARGET);
                    FSM[(uint16_t)exec_state_] = false;
                    return;
                }// 检查原轨迹是否碰撞，若碰撞则重新规划
                else if(!replanSafeCheck())
                {
                    ROS_WARN("the trajectory collision!");
                    changeState(FSM_EXEC_STATE::REPLAN_TRAJ);
                    FSM[(uint16_t)exec_state_] = false;
                    return;
                }
                return;
            } else if((target_pt_ - _vehicle_pose_).norm() < config.replan.target_replan_thresh){
                FSM[(uint16_t)exec_state_] = true;
                return;
            } else if((start_pt_ - _vehicle_pose_).norm() < config.replan.start_replan_thresh){
                FSM[(uint16_t)exec_state_] = true;
                return;
            } 
            else{
                ROS_INFO("else");
                changeState(FSM_EXEC_STATE::REPLAN_TRAJ);
                FSM[(uint16_t)exec_state_] = false;
                return;
            }
            break;
        }
    }
}

void PlanFSM::controlCallback(const ros::TimerEvent &e)
{
    switch (exec_state_)
    {
        case FSM_EXEC_STATE::TAKEOFF:
        {
            ROS_INFO("Control TAKEOFF");
            if(FSM[(uint16_t)exec_state_] == false)
                return;
            takeoff_position_.position.x = 0;
            takeoff_position_.position.y = 0;
            takeoff_position_.position.z = config.copter.hover_height;
            ROS_INFO_STREAM("the takeoff position is " << takeoff_position_.position);
            control_pub_.publish(takeoff_position_);
            ROS_INFO("takeoff_position");
            break;
        }
        case FSM_EXEC_STATE::INIT:
        {
            takeoff_position_.position.x = 0;
            takeoff_position_.position.y = 0;
            takeoff_position_.position.z = config.copter.hover_height;
            control_pub_.publish(takeoff_position_);
            break;
        }
        case FSM_EXEC_STATE::WAIT_TARGET:   // 悬停
        {
            if(FSM[(uint16_t)exec_state_] == false)
                return;
            control_pub_.publish(hover_position_);  // 悬停
            break;
        }
        case FSM_EXEC_STATE::GEN_NEW_TRAJ:
        {
            if(!success_traj_gen_)
                control_pub_.publish(hover_position_);
            break;
        }
        case FSM_EXEC_STATE::REPLAN_TRAJ:
        {
            double t_delta = config.replan.dt;
            double t = t_delta * traj_time_count_;
            if(t > trajectory_gen_.getPieceTime(traj_piece_count_))
            {
                traj_time_count_ = 0;
                t = 0;
                traj_piece_count_++;
                if(traj_piece_count_ >= trajectory_gen_.getPieceNum())
                {
                    traj_piece_count_ = trajectory_gen_.getPieceNum() - 1;
                    traj_time_count_ = std::ceil(trajectory_gen_.getPieceTime(traj_piece_count_) / t_delta);
                    t = trajectory_gen_.getPieceTime(traj_piece_count_);
                }
            }

            ROS_INFO_STREAM("traj_time_count_ is " << traj_time_count_);
            ROS_INFO_STREAM("traj_piece_count_ is " << traj_piece_count_);

            Eigen::Vector3d pos, vel, acc, pos_next, pos_last, yaw;

            getPos(t, trajectory_gen_, traj_piece_count_, pos);
            getVel(t, trajectory_gen_, traj_piece_count_, vel);
            getAcc(t, trajectory_gen_, traj_piece_count_, acc);
            /* 生成控制信号 */
            /* 位置 */
            traj_poistion_.position.x = pos.x();
            traj_poistion_.position.y = pos.y();
            traj_poistion_.position.z = pos.z();
            /* 速度 */
            traj_poistion_.velocity.x = vel.x();
            traj_poistion_.velocity.y = vel.y();
            traj_poistion_.velocity.z = vel.z();
            /* 加速度 */
            traj_poistion_.acceleration_or_force.x = acc.x();
            traj_poistion_.acceleration_or_force.y = acc.y();
            traj_poistion_.acceleration_or_force.z = acc.z();

            u_int32_t count = traj_time_count_;
            if(t_delta * (count + 1) <= trajectory_gen_.getPieceTime(traj_piece_count_))
            {
                getPos(t_delta * (count + 1), trajectory_gen_, traj_piece_count_, pos_next);
                yaw = pos_next - pos;
                traj_poistion_.yaw = atan2(yaw.y(), yaw.x());
                yaw_ = traj_poistion_.yaw;
            }
            else{
                ROS_INFO("ELSE YAW");
                ROS_INFO_STREAM("t - t_delta = " << t - t_delta);
                ROS_INFO_STREAM("t = " << t);
                double last_t = t - t_delta;
                if(t - t_delta < 0)
                {
                    last_t = 0;
                }
                getPos(last_t, trajectory_gen_, traj_piece_count_, pos_last);
                yaw = pos - pos_last;
                traj_poistion_.yaw = atan2(yaw.y(), yaw.x());
                yaw_ = traj_poistion_.yaw;
            }

            control_pub_.publish(traj_poistion_);
            traj_time_count_++;
            break;
        }
        case FSM_EXEC_STATE::EXEC_TRAJ:
        {
            double t_delta = config.replan.dt;
            double t = t_delta * traj_time_count_;
            if(t > trajectory_gen_.getPieceTime(traj_piece_count_))
            {
                traj_time_count_ = 0;
                t = 0;
                traj_piece_count_++;
                if(traj_piece_count_ >= trajectory_gen_.getPieceNum())
                {
                    traj_piece_count_ = trajectory_gen_.getPieceNum() - 1;
                    traj_time_count_ = std::ceil(trajectory_gen_.getPieceTime(traj_piece_count_) / t_delta);
                    t = trajectory_gen_.getPieceTime(traj_piece_count_);
                }
            }

            ROS_INFO_STREAM("traj_time_count_ is " << traj_time_count_);
            ROS_INFO_STREAM("traj_piece_count_ is " << traj_piece_count_);

            Eigen::Vector3d pos, vel, acc, pos_next, pos_last, yaw;

            getPos(t, trajectory_gen_, traj_piece_count_, pos);
            getVel(t, trajectory_gen_, traj_piece_count_, vel);
            getAcc(t, trajectory_gen_, traj_piece_count_, acc);
            /* 生成控制信号 */
            /* 位置 */
            traj_poistion_.position.x = pos.x();
            traj_poistion_.position.y = pos.y();
            traj_poistion_.position.z = pos.z();
            /* 速度 */
            traj_poistion_.velocity.x = vel.x();
            traj_poistion_.velocity.y = vel.y();
            traj_poistion_.velocity.z = vel.z();
            /* 加速度 */
            traj_poistion_.acceleration_or_force.x = acc.x();
            traj_poistion_.acceleration_or_force.y = acc.y();
            traj_poistion_.acceleration_or_force.z = acc.z();

            u_int32_t count = traj_time_count_;
            if(t_delta * (count + 1) <= trajectory_gen_.getPieceTime(traj_piece_count_))
            {
                getPos(t_delta * (count + 1), trajectory_gen_, traj_piece_count_, pos_next);
                yaw = pos_next - pos;
                traj_poistion_.yaw = atan2(yaw.y(), yaw.x());
                yaw_ = traj_poistion_.yaw;
            }
            else{
                ROS_INFO_STREAM("t - t_delta = " << t - t_delta);
                ROS_INFO_STREAM("t = " << t);
                double last_t = t - t_delta;
                if(t - t_delta < 0)
                {
                    last_t = 0;
                }
                getPos(last_t, trajectory_gen_, traj_piece_count_, pos_last);
                yaw = pos - pos_last;
                traj_poistion_.yaw = atan2(yaw.y(), yaw.x());
                yaw_ = traj_poistion_.yaw;
            }

            control_pub_.publish(traj_poistion_);
            traj_time_count_++;
            break;
        }
    }
}

}
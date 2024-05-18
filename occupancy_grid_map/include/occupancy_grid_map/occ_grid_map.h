#pragma once

#include <vector>
#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

namespace CUADC{

enum VoxelState{
    FREE=0u,
    OCCUPANY=1u
};

struct InflateVoxel
{
    VoxelState state = VoxelState::FREE;
    uint32_t count = 0;
    InflateVoxel(VoxelState s, uint32_t c):state(s), count(c){};
};

double logit(double x)
{
    return log(x / (1 - x));
}

class Map;
typedef Map* MapPtr;

struct Camera
{
    Eigen::Isometry3d T_bc_;        // 相机坐标系变为机体坐标系
    Eigen::Matrix3d Intrinsis_;     // 相机内参矩阵
    
    int cam_depth_width_,cam_depth_height_;
    double fx_, fy_, cx_, cy_;

    double depth_maxdist_, depth_minidist_;
    double max_ray_length_;

    int depth_filter_margin_;
    int skip_pixel_;
    
    double k_depth_scaling_factor_;

    Eigen::Vector3d camera_position_, last_camera_position_;        // 相机位置
    Eigen::Quaterniond camera_q_, last_camera_q_;        // 相机姿态
    cv::Mat depth_image_, last_depth_image_;        // 前后两帧的深度图
    
    Eigen::Isometry3d T_wc_;
    Eigen::Isometry3d T_cw_;
    Eigen::Matrix4d cam2body_;

    std::vector<Eigen::Vector3d> proj_points_;
    std::vector<Eigen::Vector2i> proj_invaild_;
    std::vector<Eigen::Vector3d> ray_point_set;
    uint32_t proj_points_cnt_;

    Camera()
    {
        Eigen::Matrix3d R_bc_;
        R_bc_ << 0, 0, 1,
                -1, 0, 0,
                0, -1, 0; 
        T_bc_.rotate(R_bc_);
        T_bc_.pretranslate(Eigen::Vector3d(-0.02, 0, 0));
        Intrinsis_ = Eigen::Matrix3d::Zero();
        T_wc_ = Eigen::Isometry3d::Identity();
    }
};

struct MapParamters
{
    Eigen::Vector3d map_origin_, map_size_; // 地图原点，地图大小
    Eigen::Vector3d map_min_boundary_, map_max_boundary_;   // 地图边界
    Eigen::Vector3i map_voxel_num_;     // 体素数量
    double resolution_, resolution_inv_;    // 分辨率(0-1)
    double obstacles_inflation_;
    double ground_height_;

    std::string frame_id_;
    double p_occ_max_, p_occ_min_;      // 每个格子的最大概率 
    double prob_max_logit_, prob_min_logit_;  // 将每个格子的概率转换成logit

    double lofree_, looccu_;    // 每次观测后的增量
    bool occ_need_update_;
};

struct OccupanyGridMap
{
    std::vector<double> occupany_map_;      // 原始栅格占用地图，用后验概率表示，当大于0.5时认为被占用
    std::vector<VoxelState> occupany_map_inflate;     // 膨胀地图，1代表占用，0代表空闲
    std::vector<InflateVoxel> occupany_map_inflate_;     // 膨胀地图，1代表占用，0代表空闲
};

class Map
{
public:
    Map() {}
    ~Map() {}

    void initMap(ros::NodeHandle &nh);

    inline int toAddress(const Eigen::Vector3i &index);

    inline int toAddress(const Eigen::Vector3d &pos);

    inline void pos2Index(const Eigen::Vector3d &pos, Eigen::Vector3i &id);

    inline void index2Pos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);

    inline bool isInMap(const Eigen::Vector3d& pos);
    
    inline bool PointBound(Eigen::Vector3i& id);

    VoxelState isOccupied(const Eigen::Vector3i& id){
        return occ_map_.occupany_map_inflate_.at(toAddress(id)).state;
    }

    double getCamMaxRayLength(){return cam_.max_ray_length_;}
private:

    void projectDepthImage();

    void occupanyProb();

    void setRayProb(const std::vector<Eigen::Vector3d>  &ray_point);

    void setInvaildRayProb(const std::vector<Eigen::Vector2i>  &ray_cam_point);

    inline void setOccupany(const Eigen::Vector3d &pos, VoxelState state);

    inline void setOccupany(const Eigen::Vector3i &id, VoxelState state);

    inline void inflatePoint(const Eigen::Vector3i& pt, VoxelState state);

    void raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, std::vector<Eigen::Vector3d> &ray);

    void updateOccupancyCallback(const ros::TimerEvent& /*event*/);

    void visualCallback(const ros::TimerEvent& );

    void publishMap();

    void publishInflateMap();

    void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
    
public:
    MapParamters mp_;
private:
    Camera cam_;
    OccupanyGridMap occ_map_;

    ros::NodeHandle nh_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
        SyncPolicyImageOdom;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

    ros::Publisher map_pub_, map_inf_pub_;
    ros::Timer occ_timer_, vis_timer_;

    SynchronizerImageOdom sync_image_odom_;

    // point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    pcl::PointCloud<pcl::PointXYZ> cloud_inflate_;
};

inline int Map::toAddress(const Eigen::Vector3i &index){
    return mp_.map_voxel_num_(2) * mp_.map_voxel_num_(1) * index(0) + mp_.map_voxel_num_(2) * index(1) + index(2);
}

inline int Map::toAddress(const Eigen::Vector3d &pos){
    Eigen::Vector3i id;
    pos2Index(pos, id);
    return toAddress(id);
}

inline void Map::pos2Index(const Eigen::Vector3d &pos, Eigen::Vector3i &id){
    for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void Map::index2Pos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = ((double)id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline bool Map::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline void Map::setOccupany(const Eigen::Vector3d &pos, VoxelState state)
{
    Eigen::Vector3i id;
    pos2Index(pos, id);
    int address = toAddress(id);
    if(VoxelState::FREE == state)
    {
        if(occ_map_.occupany_map_.at(address) > mp_.prob_min_logit_)
            occ_map_.occupany_map_.at(address) += mp_.lofree_;
        else if(occ_map_.occupany_map_.at(address) < mp_.prob_min_logit_)
            occ_map_.occupany_map_.at(address) = mp_.prob_min_logit_;
    }
    else if(VoxelState::OCCUPANY == state)
    {
        // if(occ_map_.occupany_map_.at(address) < mp_.prob_max_logit_)
            occ_map_.occupany_map_.at(address) += mp_.looccu_;
        // else if(occ_map_.occupany_map_.at(address) >= mp_.prob_max_logit_)
        //     occ_map_.occupany_map_.at(address) = mp_.prob_max_logit_;
    }
}

inline void Map::setOccupany(const Eigen::Vector3i &id, VoxelState state)
{
    int address = toAddress(id);
    if(VoxelState::FREE == state)
    {
        if(occ_map_.occupany_map_.at(address) > mp_.prob_min_logit_)
            occ_map_.occupany_map_.at(address) += mp_.lofree_;
        else if(occ_map_.occupany_map_.at(address) <= mp_.prob_min_logit_)
            occ_map_.occupany_map_.at(address) = mp_.prob_min_logit_;
    }
    else if(VoxelState::OCCUPANY == state)
    {
        // if(occ_map_.occupany_map_.at(address) < mp_.prob_max_logit_)
            occ_map_.occupany_map_.at(address) += mp_.looccu_;
        // else if(occ_map_.occupany_map_.at(address) >= mp_.prob_max_logit_)
        //     occ_map_.occupany_map_.at(address) = mp_.prob_max_logit_;
    }
}

inline void Map::inflatePoint(const Eigen::Vector3i& pt, VoxelState state)
{
    // ROS_INFO("inflatePoint begin");
    if(VoxelState::FREE == state)
    {
        occ_map_.occupany_map_inflate_.at(toAddress(pt)).state = VoxelState::FREE;
        return;
    }

    Eigen::Vector3i id;
    int inf_step = ceil(mp_.obstacles_inflation_ * mp_.resolution_inv_);
    for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
            for (int z = -inf_step; z <= inf_step; ++z) {
                id << pt(0) + x, pt(1) + y, pt(2) + z;
                PointBound(id);
                // if(VoxelState::FREE == state)
                // {
                //     if(occ_map_.occupany_map_inflate_.at(toAddress(id)).count > 1)
                //         occ_map_.occupany_map_inflate_.at(toAddress(id)).count--;
                //     else
                //     {
                //         occ_map_.occupany_map_inflate_.at(toAddress(id)).count = 0;
                //         occ_map_.occupany_map_inflate_.at(toAddress(id)).state = VoxelState::FREE;
                //     }
                // }
                // else
                // {
                //     occ_map_.occupany_map_inflate_.at(toAddress(id)).count++;
                    occ_map_.occupany_map_inflate_.at(toAddress(id)).state = VoxelState::OCCUPANY;
                // }
                // occ_map_.occupany_map_inflate.at(toAddress(id)) = state;
                // ROS_INFO_STREAM("inflatePoint point is " << toAddress(id));
            }
    // ROS_INFO("inflatePoint end");
}


inline bool Map::PointBound(Eigen::Vector3i& id)
{
    //  check x
    bool sign = true;
    if(id.x() >= mp_.map_voxel_num_.x())
    {
        id(0) = mp_.map_voxel_num_.x() - 1;
        sign = false;
    }
    else if(id.x() < 0)
    {
        id(0) = 0;
        sign = false;
    }
    //  check y
    if(id.y() >= mp_.map_voxel_num_.y())
    {
        id(1) = mp_.map_voxel_num_.y() - 1;
        sign = false;
    }
    else if(id.y() < 0)
    {
        id(1) = 0;
        sign = false;
    }
    //  check z
    if(id.z() >= mp_.map_voxel_num_.z())
    {    
        id(2) = mp_.map_voxel_num_.z() - 1;
        sign = false;
    }
    else if(id.z() < 0)
    {
        id(2) = 0;
        sign = false;
    }
    return sign;
}

}
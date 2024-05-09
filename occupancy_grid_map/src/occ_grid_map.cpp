#include "occupancy_grid_map/occ_grid_map.h"
#include "occupancy_grid_map/raycast.h"

namespace CUADC{

void Map::initMap(ros::NodeHandle &nh)
{
    nh_ = nh;
    double x_size, y_size, z_size;
    nh_.param("occupany_grid_map/resolution", mp_.resolution_, -1.0);
    nh_.param("occupany_grid_map/map_size_x", x_size, -1.0);
    nh_.param("occupany_grid_map/map_size_y", y_size, -1.0);
    nh_.param("occupany_grid_map/map_size_z", z_size, -1.0);
    nh_.param("occupany_grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);
    nh_.param("occupany_grid_map/ground_heigh", mp_.ground_height_);
    nh_.param("occupany_grid_map/cam_depth_width", cam_.cam_depth_width_, 640);
    nh_.param("occupany_grid_map/cam_depth_height", cam_.cam_depth_height_, 480);
    nh_.param("occupany_grid_map/k_depth_scaling_factor", cam_.k_depth_scaling_factor_, -1.0);
    nh_.param("occupany_grid_map/fx",cam_.fx_, -1.0);
    nh_.param("occupany_grid_map/fy",cam_.fy_, -1.0);
    nh_.param("occupany_grid_map/cx",cam_.cx_, -1.0);
    nh_.param("occupany_grid_map/cy",cam_.cy_, -1.0);
    nh_.param("occupany_grid_map/depth_maxdist", cam_.depth_maxdist_, -1.0);
    nh_.param("occupany_grid_map/depth_minidist", cam_.depth_minidist_, -1.0);
    nh_.param("occupany_grid_map/max_ray_length", cam_.max_ray_length_, -1.0);
    nh_.param("occupany_grid_map/depth_filter_margin", cam_.depth_filter_margin_, -1);
    nh_.param("occupany_grid_map/skip_pixel", cam_.skip_pixel_, -1);
    nh_.param("occupany_grid_map/p_occ_max", mp_.p_occ_max_, 0.7);
    nh_.param("occupany_grid_map/p_occ_min", mp_.p_occ_min_, 0.12);
    nh_.param("occupany_grid_map/lofree", mp_.lofree_, -0.65);
    nh_.param("occupany_grid_map/looccu", mp_.looccu_, 0.65);
    nh_.param("occupany_grid_map/frame_id", mp_.frame_id_, std::string("world"));

    // paramters init
    mp_.resolution_inv_ = 1.0 / mp_.resolution_;
    mp_.map_origin_ = Eigen::Vector3d(-x_size / 2, -y_size / 2, mp_.ground_height_);
    mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
    mp_.prob_max_logit_ = logit(mp_.p_occ_max_);
    mp_.prob_min_logit_ = logit(mp_.p_occ_min_);

    ROS_INFO_STREAM("max logit " << mp_.prob_max_logit_);
    ROS_INFO_STREAM("min logit " << mp_.prob_min_logit_);

    for(uint i = 0; i < 3; ++i)
        mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);
    
    mp_.map_min_boundary_ = mp_.map_origin_;
    mp_.map_max_boundary_ = mp_.map_size_ + mp_.map_origin_;

    int voxel_num = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
    occ_map_.occupany_map_ = std::vector<double>(voxel_num, mp_.prob_min_logit_);
    occ_map_.occupany_map_inflate = std::vector<VoxelState>(voxel_num, VoxelState::FREE);

    cam_.proj_points_.resize(cam_.cam_depth_width_ * cam_.cam_depth_height_ / cam_.skip_pixel_ / cam_.skip_pixel_);
    
    cam_.Intrinsis_ << cam_.fx_, 0, cam_.cx_,
                       0, cam_.fy_, cam_.cy_,
                       0, 0, 1;

    // ros init
    ROS_INFO("ros init");
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/iris_0/realsense/depth_camera/depth/image_raw", 10));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/iris_0/mavros/odometry/in", 10));
    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&Map::depthOdomCallback, this, _1, _2));

    // ros publisher
    ROS_INFO("ros publisher");
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupany_grid_map/grid_map", 10);
    map_inf_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("occupany_grid_map/inflate_grid_map", 10);

    // ros Timer
    ROS_INFO("ros Timer");
    occ_timer_ = nh_.createTimer(ros::Duration(0.05), &Map::updateOccupancyCallback, this);
    vis_timer_ = nh.createTimer(ros::Duration(0.1), &Map::visualCallback, this);

    mp_.occ_need_update_ = false;
    cam_.proj_points_cnt_ = 0;
    cam_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, -0.02,
      0.0, 0.0, 0.0, 1.0;
}

void Map::projectDepthImage()
{
    cam_.proj_invaild_.clear();
    cam_.proj_points_cnt_ = 0;
    int cols = cam_.depth_image_.cols;
    int rows = cam_.depth_image_.rows;

    Eigen::Matrix3d camera_r = cam_.camera_q_.toRotationMatrix();

    Eigen::Vector3d proj_pt;
    Eigen::Vector2i proj_invaild;
    // ROS_INFO("projectDepthImage begin");
    uint16_t *row_ptr;
    for(int v = cam_.depth_filter_margin_; v < rows - cam_.depth_filter_margin_; v += cam_.skip_pixel_)
    {
        row_ptr = cam_.depth_image_.ptr<uint16_t>(v) + cam_.depth_filter_margin_;

        for(int u = cam_.depth_filter_margin_; u < cols - cam_.depth_filter_margin_; u += cam_.skip_pixel_)
        {
            uint16_t d = (*row_ptr) / cam_.k_depth_scaling_factor_;
            row_ptr = row_ptr + cam_.skip_pixel_;
            // if(*row_ptr == 0)
            // {
            //     d = cam_.depth_maxdist_;
            // }

            if(d == 0 || d <= cam_.depth_minidist_)  
            {
                proj_invaild(0) = u;
                proj_invaild(1) = v;
                cam_.proj_invaild_.push_back(proj_invaild);
                continue;
            }
            else if(d > cam_.depth_maxdist_)
                d = cam_.depth_maxdist_;
            // camera坐标系
            proj_pt(0) = (u - cam_.cx_) * d / cam_.fx_;
            proj_pt(1) = (v - cam_.cy_) * d / cam_.fy_;
            proj_pt(2) = d;
            // 转换到world坐标系
            proj_pt = camera_r * proj_pt + cam_.camera_position_;

            if(!isInMap(proj_pt))   // 在地图之外
            {
                proj_invaild(0) = u;
                proj_invaild(1) = v;
                cam_.proj_invaild_.push_back(proj_invaild);
                continue;
            }
            cam_.proj_points_[cam_.proj_points_cnt_++] = proj_pt;
            // if (u == 320 && v == 240)
            // ROS_INFO_STREAM("depth : " << d);
        }
    }
    ROS_INFO_STREAM("the size of proj_points is " << cam_.proj_points_cnt_);
}

// 计算占用概率
void Map::occupanyProb()
{
    // ROS_INFO("occupanyProb begin");
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d pt_w, ray_pt;
    Eigen::Vector3i id;
    for(uint32_t i = 0; i < cam_.proj_points_cnt_; ++i)
    {
        pt_w = cam_.proj_points_.at(i);
        Raycast(pt_w / mp_.resolution_, cam_.camera_position_ / mp_.resolution_, mp_.map_min_boundary_  / mp_.resolution_, mp_.map_max_boundary_ / mp_.resolution_, cam_.ray_point_set);
        for(auto &p:cam_.ray_point_set)
        {
            Eigen::Vector3d tmp = (p + half) * mp_.resolution_;
            setOccupany(tmp, VoxelState::FREE);
            pos2Index(tmp, id);
            PointBound(id);
            if(occ_map_.occupany_map_.at(toAddress(id)) >= mp_.prob_max_logit_)
                inflatePoint(id, VoxelState::OCCUPANY);
            else if(occ_map_.occupany_map_.at(toAddress(id)) < mp_.prob_max_logit_)// 取消膨胀
                inflatePoint(id, VoxelState::FREE);
        }
        setOccupany(pt_w, VoxelState::OCCUPANY);
        pos2Index(pt_w, id);
        PointBound(id);
        // ROS_INFO_STREAM("the id is " << id);
        if(occ_map_.occupany_map_.at(toAddress(id)) >= mp_.prob_max_logit_)
            inflatePoint(id, VoxelState::OCCUPANY);
        else if(occ_map_.occupany_map_.at(toAddress(id)) < mp_.prob_max_logit_)// 取消膨胀
            inflatePoint(id, VoxelState::FREE);
    }
    // ROS_INFO("occupanyProb end");
    // setInvaildRayProb(cam_.proj_invaild_);
    // ROS_INFO("occupanyProb loop end");
}

// 计算占用概率
// void Map::occupanyProb()
// {
//     setOccupany(cam_.camera_position_, VoxelState::FREE);
//     std::vector<Eigen::Vector3d> ray_point;
//     Eigen::Vector3i id, ray_id;
//     // for(auto &pt_w:cam_.proj_points_)
//     Eigen::Vector3d pt_w;
//     for(uint32_t i = 0; i < cam_.proj_points_cnt_; ++i)
//     {
//         pt_w = cam_.proj_points_.at(i);
//         ray_point.clear();
//         raycast(cam_.camera_position_, pt_w, ray_point);
//         if(ray_point.empty())
//             continue;
//         // 设置概率
//         setRayProb(ray_point);
//         // 膨胀
//         for(auto &ray_p:ray_point)
//         {
//             pos2Index(ray_p, ray_id);
//             if(occ_map_.occupany_map_.at(toAddress(ray_id)) >= mp_.prob_max_logit_)
//                 inflatePoint(ray_id, VoxelState::OCCUPANY);
//             else if(occ_map_.occupany_map_.at(toAddress(ray_id)) < mp_.prob_max_logit_) // 取消膨胀
//                 inflatePoint(ray_id, VoxelState::FREE);
//         }
//         setOccupany(pt_w, VoxelState::OCCUPANY);
//         // 生成膨胀地图
//         pos2Index(pt_w, id);
//         if(occ_map_.occupany_map_.at(toAddress(id)) >= mp_.prob_max_logit_)
//             inflatePoint(id, VoxelState::OCCUPANY);
//         else if(occ_map_.occupany_map_.at(toAddress(id)) < mp_.prob_max_logit_) // 取消膨胀
//             inflatePoint(id, VoxelState::FREE);
//     }
// }

void Map::setRayProb(const std::vector<Eigen::Vector3d>  &ray_point)
{
    for(auto &p:ray_point)
        setOccupany(p, VoxelState::FREE);
}

// void Map::setInvaildRayProb(const std::vector<Eigen::Vector2i>  &ray_cam_point)
// {
//     Eigen::Vector3d p_w, p_diect_w, p_ray;
//     Eigen::Vector3i id_current, id_last;
//     bool id_sign = true;
//     Eigen::Matrix3d camera_r = cam_.camera_q_.toRotationMatrix();
//     // ROS_INFO_STREAM("p_ray is " << p_ray);
//     // ROS_INFO_STREAM("the camera position is " << cam_.camera_position_);
//     for(auto &p_c:ray_cam_point)
//     {
//         p_ray << cam_.camera_position_.x(), cam_.camera_position_.y(), cam_.camera_position_.z();
//         p_w(0) = ((double)p_c(0) - cam_.cx_) / cam_.fx_;
//         p_w(1) = ((double)p_c(1) - cam_.cy_) / cam_.fy_;
//         p_w(2) = 1;
//         p_diect_w = camera_r * p_w + cam_.camera_position_;
//         p_diect_w.normalize();
//         pos2Index(cam_.camera_position_, id_last);
//         id_current = id_last;
//         id_sign = true;
//         // ROS_INFO_STREAM("isInMap(p_ray)" << isInMap(p_ray));
//         // ROS_INFO_STREAM("loop p_ray is " << p_ray);
//         // ROS_INFO_STREAM("the loop camera position is " << cam_.camera_position_);
//         // ROS_INFO_STREAM("the min boundary is " << mp_.map_min_boundary_);
//         // ROS_INFO_STREAM("the max boundary is " << mp_.map_max_boundary_);
//         while(isInMap(p_ray))
//         {
//             ROS_INFO("setInvaildRayProb loop begin");
//             if(id_sign)
//             {
//                 id_sign = false;
//                 p_ray += mp_.resolution_ * p_diect_w;
//                 if(PointBound(id_last))
//                     break;

//                 setOccupany(id_last, VoxelState::FREE);
//                 if(occ_map_.occupany_map_.at(toAddress(id_current)) >= mp_.prob_max_logit_)
//                     inflatePoint(id_current, VoxelState::OCCUPANY);
//                 else if(occ_map_.occupany_map_.at(toAddress(id_current)) < mp_.prob_max_logit_) // 取消膨胀
//                     inflatePoint(id_current, VoxelState::FREE);
//             }
//             else// 更新
//             {
//                 id_last = id_current;
//                 p_ray += 2 * mp_.resolution_ * p_diect_w;
//                 pos2Index(p_ray, id_current);
//                 ROS_INFO_STREAM("id " << '\n' << id_current);
//                 if(PointBound(id_current))
//                     break;

//                 setOccupany(id_current, VoxelState::FREE);
//                 if(occ_map_.occupany_map_.at(toAddress(id_current)) >= mp_.prob_max_logit_)
//                     inflatePoint(id_current, VoxelState::OCCUPANY);
//                 else if(occ_map_.occupany_map_.at(toAddress(id_current)) < mp_.prob_max_logit_) // 取消膨胀
//                     inflatePoint(id_current, VoxelState::FREE);
//             }
            
//             if(id_current == id_last)
//                 continue;


//         }
//     }
// }


void Map::setInvaildRayProb(const std::vector<Eigen::Vector2i>  &ray_cam_point)
{
    Eigen::Vector3d p_w, p_we, p_ray;
    Eigen::Vector3i id;
    Eigen::Vector3d tmp, half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Matrix3d camera_r = cam_.camera_q_.toRotationMatrix();
    for(auto &p_c:ray_cam_point)
    {
        p_ray << cam_.camera_position_.x(), cam_.camera_position_.y(), cam_.camera_position_.z();
        p_w(0) = ((double)p_c(0) - cam_.cx_) * cam_.max_ray_length_ / cam_.fx_;
        p_w(1) = ((double)p_c(1) - cam_.cy_) * cam_.max_ray_length_ / cam_.fy_;
        p_w(2) = cam_.max_ray_length_;
        p_we = camera_r * p_w + cam_.camera_position_;
        Raycast(p_we / mp_.resolution_, cam_.camera_position_ / mp_.resolution_, mp_.map_min_boundary_  / mp_.resolution_, mp_.map_max_boundary_ / mp_.resolution_, cam_.ray_point_set);
        // ROS_INFO_STREAM("setInvaildRayProb ray_point_set size is " << cam_.ray_point_set.size());
        for(auto &p:cam_.ray_point_set)
        {
            tmp = (p + half) * mp_.resolution_;
            setOccupany(tmp, VoxelState::FREE);
            pos2Index(tmp, id);
            PointBound(id);
            // ROS_INFO("inflatePoint");
            if(occ_map_.occupany_map_.at(toAddress(id)) >= mp_.prob_max_logit_)
                inflatePoint(id, VoxelState::OCCUPANY);
            else if(occ_map_.occupany_map_.at(toAddress(id)) < mp_.prob_max_logit_)// 取消膨胀
                inflatePoint(id, VoxelState::FREE);
            
        }
    }
    // ROS_INFO_STREAM("setInvaildRayProb end");
}

void Map::raycast(const Eigen::Vector3d &start, const Eigen::Vector3d &end, std::vector<Eigen::Vector3d> &ray)
{  
    Eigen::Vector3d direct = (end - start).normalized();
    Eigen::Vector3d ray_point;
    double distance = (end - start).norm();
    int piece_num = floor(distance * mp_.resolution_inv_) - 1;
    for(int i = piece_num; i >= 0; --i)
    {
        if((ray_point - end).norm() < 0.01)
            break;
        ray_point = i * mp_.resolution_ * direct + cam_.camera_position_;
        ray.push_back(ray_point);
    }
}

// 更新地图
void Map::updateOccupancyCallback(const ros::TimerEvent& /*event*/)
{
    if(!mp_.occ_need_update_)
        return;
    
    // 进行投影
    projectDepthImage();

    // 计算占用概率
    if(cam_.proj_points_cnt_)
        occupanyProb();

    mp_.occ_need_update_ = false;
}

void Map::visualCallback(const ros::TimerEvent& )
{
    publishMap();

    publishInflateMap();
}

void Map::publishMap()
{
    // ROS_INFO("publishMap begin");
    if(map_pub_.getNumSubscribers() <= 0)
        return;
    pcl::PointXYZ pt;
    cloud_.clear();
    Eigen::Vector3i id;
    Eigen::Vector3d pos;
    for(int x = 0; x < mp_.map_voxel_num_.x(); ++x)
        for(int y = 0; y < mp_.map_voxel_num_.y(); ++y)
            for(int z = 0; z < mp_.map_voxel_num_.z(); ++z)
            {
                id << x, y, z;
                PointBound(id);
                if(occ_map_.occupany_map_.at(toAddress(id)) >= mp_.prob_max_logit_)
                {
                    ROS_INFO_STREAM("publishMap loop");
                    index2Pos(id, pos);
                    pt.x = pos.x();
                    pt.y = pos.y();
                    pt.z = pos.z();
                    cloud_.push_back(pt);
                }
            }

    cloud_.width = cloud_.points.size();
    cloud_.height = 1;
    cloud_.is_dense = true;
    cloud_.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud_, cloud_msg);
    map_pub_.publish(cloud_msg);
    // ROS_INFO("publishMap end");
}


void Map::publishInflateMap()
{
    // ROS_INFO("publishInflateMap begin");
    if(map_inf_pub_.getNumSubscribers() <= 0)
        return;

    pcl::PointXYZ pt;
    cloud_inflate_.clear();

    Eigen::Vector3i id;
    Eigen::Vector3d pos;
    for(int x = 0; x < mp_.map_voxel_num_.x(); ++x)
        for(int y = 0; y < mp_.map_voxel_num_.y(); ++y)
            for(int z = 0; z < mp_.map_voxel_num_.z(); ++z)
            {
                id << x, y, z;
                PointBound(id);
                if(occ_map_.occupany_map_inflate.at(toAddress(id)) == VoxelState::FREE)
                    continue;
                index2Pos(id, pos);
                pt.x = pos.x();
                pt.y = pos.y();
                pt.z = pos.z();
                cloud_inflate_.push_back(pt);
            }

    cloud_inflate_.width = cloud_inflate_.points.size();
    cloud_inflate_.height = 1;
    cloud_inflate_.is_dense = true;
    cloud_inflate_.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg_inflate;

    pcl::toROSMsg(cloud_inflate_, cloud_msg_inflate);
    map_inf_pub_.publish(cloud_msg_inflate);
    // ROS_INFO("publishInflateMap end");
}

void Map::depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom)
{
    // ROS_INFO("depthOdomCallback");
    Eigen::Quaterniond q_wb = Eigen::Quaterniond(odom->pose.pose.orientation.w, 
                                                 odom->pose.pose.orientation.x, 
                                                 odom->pose.pose.orientation.y, 
                                                 odom->pose.pose.orientation.z);

    // body --> world
    Eigen::Isometry3d T_wb;
    T_wb.rotate(q_wb);
    T_wb.pretranslate(Eigen::Vector3d(odom->pose.pose.position.x, 
                                      odom->pose.pose.position.y,
                                      odom->pose.pose.position.z));
    
    // camera --> world
    cam_.T_wc_ = T_wb * cam_.T_bc_;
    cam_.camera_position_ = cam_.T_wc_.translation();
    cam_.T_cw_ = cam_.T_wc_.inverse();


    Eigen::Matrix3d body_r_m = q_wb.toRotationMatrix();
    Eigen::Matrix4d body2world;
    body2world.block<3, 3>(0, 0) = body_r_m;
    body2world(0, 3) = odom->pose.pose.position.x;
    body2world(1, 3) = odom->pose.pose.position.y;
    body2world(2, 3) = odom->pose.pose.position.z;
    body2world(3, 3) = 1.0;

    Eigen::Matrix4d cam_T = body2world * cam_.cam2body_;
    cam_.camera_position_(0) = cam_T(0, 3);
    cam_.camera_position_(1) = cam_T(1, 3);
    cam_.camera_position_(2) = cam_T(2, 3);
    cam_.camera_q_ = Eigen::Quaterniond(cam_T.block<3, 3>(0, 0));
    // ROS_INFO_STREAM("the cam_t is " << cam_T);
    // ROS_INFO_STREAM("T_wc_ is " << cam_.T_wc_.matrix());
    /* 获取深度图 */
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, cam_.k_depth_scaling_factor_);
    }
    cv_ptr->image.copyTo(cam_.depth_image_);
    mp_.occ_need_update_ = true;
}

} // namspace CUADC
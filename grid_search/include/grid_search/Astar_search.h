#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "occupancy_grid_map/occ_grid_map.h"
#include "node.h"

// debug
#include "backward.hpp"

namespace CUADC{

class AstarPathFinder
{
protected:
    const MapPtr map_;  // read only
    GridNodePtr *** GridNodeMap_;   // 地图
    Eigen::Vector3i goalIdx_;
    Eigen::Vector3i startIdx_;
    Eigen::Vector3d start_pt_;  // 起始点

    GridNodePtr terminatePtr_;


    std::multimap<double, GridNodePtr> openSet_;
    // 对角距离启发函数
    double getHeu(GridNodePtr node1, GridNodePtr node2);
    // 欧式距离启发函数
    double getHeuEuclidean(GridNodePtr node1, GridNodePtr node2);
    // 曼哈顿距离启发函数
    double getHeuManhattan(GridNodePtr node1, GridNodePtr node2);
    
    // 含有Tie Breaker的启发函数
    // 对角距离启发函数
    double getHeuTieBreaker(GridNodePtr node1, GridNodePtr node2);
    // 欧式距离启发函数
    double getHeuEuclideanTieBreaker(GridNodePtr node1, GridNodePtr node2);
    // 曼哈顿距离启发函数
    double getHeuManhattanTieBreaker(GridNodePtr node1, GridNodePtr node2);

    inline void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		

    inline void resetGridMap(GridNodePtr *** GridNodeMap, std::vector<Eigen::Vector3i> &expanded);

    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

public:
    AstarPathFinder(const MapPtr map_ptr):map_(map_ptr){
        Eigen::Vector3d pos;
        GridNodeMap_ = new GridNodePtr ** [map_ptr->mp_.map_voxel_num_.x()];
        for(int i = 0; i < map_ptr->mp_.map_voxel_num_.x(); ++i){
            GridNodeMap_[i] = new GridNodePtr * [map_ptr->mp_.map_voxel_num_.y()];
            for(int j = 0; j < map_ptr->mp_.map_voxel_num_.y(); ++j){
                GridNodeMap_[i][j] = new GridNodePtr [map_ptr->mp_.map_voxel_num_.z()];
                for(int k = 0; k < map_ptr->mp_.map_voxel_num_.z(); ++k){
                    Eigen::Vector3i tmpIdx(i, j, k);
                    map_ptr->index2Pos(tmpIdx, pos);
                    GridNodeMap_[i][j][k] = new GridNode(tmpIdx, pos);
                }
            }
        }
    };
    ~AstarPathFinder(){};
    // 对角距离启发函数
    void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    // 欧式距离启发函数
    void AstarGraphSearchOfEuclidean(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    // 曼哈顿距离启发函数
    void AstarGraphSearchOfManhattan(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

    // 使用Tie Breaker启发函数的A*算法
    // 对角距离启发函数
    void AstarGraphSearchTieBreaker(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    // 欧式距离启发函数
    void AstarGraphSearchOfEuclideanTieBreaker(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    // 曼哈顿距离启发函数
    void AstarGraphSearchOfManhattanTieBreaker(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);


    Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
    std::vector<Eigen::Vector3d> getPath();
    std::vector<Eigen::Vector3d> getVisitedNodes();
    std::vector<Eigen::Vector3i> expanded_;     // 被扩展过的节点
};

} // namespace CUADC



#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <occupancy_grid_map/occ_grid_map.h>
// #include "occupancy_grid_map/occ_grid_map.h"
#include "node.h"

// debug
#include "backward.hpp"

namespace CUADC{

enum AstarHeu
{
    DIALOG=0u,
    EUCLIDEAN=1u,
    MANHATTAN=2u,
    DIALOG_TIEBREAKER=3u,
    EUCLIDEAN_TIEBREAKER=4u,
    MANHATTAN_TIEBREAKER=5u
};

class AstarPathFinder
{
protected:
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

    inline void resetGridMap(GridNodePtr *** GridNodeMap, std::vector<Eigen::Vector3i> &visited);

    inline double perpendicularDistance(const Eigen::Vector3d &p, const Eigen::Vector3d &start, const Eigen::Vector3d &end);
public:
    // AstarPathFinder(const MapPtr map_ptr):map_(map_ptr){
    //     Eigen::Vector3d pos;
    //     GridNodeMap_ = new GridNodePtr ** [map_ptr->mp_.map_voxel_num_.x()];
    //     for(int i = 0; i < map_ptr->mp_.map_voxel_num_.x(); ++i){
    //         GridNodeMap_[i] = new GridNodePtr * [map_ptr->mp_.map_voxel_num_.y()];
    //         for(int j = 0; j < map_ptr->mp_.map_voxel_num_.y(); ++j){
    //             GridNodeMap_[i][j] = new GridNodePtr [map_ptr->mp_.map_voxel_num_.z()];
    //             for(int k = 0; k < map_ptr->mp_.map_voxel_num_.z(); ++k){
    //                 Eigen::Vector3i tmpIdx(i, j, k);
    //                 map_ptr->index2Pos(tmpIdx, pos);
    //                 GridNodeMap_[i][j][k] = new GridNode(tmpIdx, pos);
    //             }
    //         }
    //     }
    // };
    AstarPathFinder(){};
    ~AstarPathFinder(){};

    void initAstarPathFinder(const MapPtr map_ptr){
        map_ = map_ptr;
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
    // Astar
    void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, AstarHeu function);

    void getPath(std::vector<Eigen::Vector3d> &path);

    void rdpPath(std::vector<Eigen::Vector3d> &path, double epsilon, std::vector<Eigen::Vector3d> &simplified);

    std::vector<Eigen::Vector3i> visited_;     // 被访问过的节点

    MapPtr map_;  
};

} // namespace CUADC



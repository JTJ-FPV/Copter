#include "grid_search/Astar_search.h"

namespace CUADC{

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    // 对角距离
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));
    double h = dx + dy + dz + (sqrt(3) - 3) * std::min(std::min(dx, dy), dz);
    return h;
}

double AstarPathFinder::getHeuEuclidean(GridNodePtr node1, GridNodePtr node2)
{
    // 这里使用欧式距离
    double h = sqrt(
        (node1->index(0) - node2->index(0)) * (node1->index(0) - node2->index(0)) +
        (node1->index(1) - node2->index(1)) * (node1->index(1) - node2->index(1)) +
        (node1->index(2) - node2->index(2)) * (node1->index(2) - node2->index(2))
    );
    return h;
}

double AstarPathFinder::getHeuManhattan(GridNodePtr node1, GridNodePtr node2)
{
    // 曼哈顿距离
    double h = abs(node1->index(0) - node2->index(0)) +
                            abs(node1->index(1) - node2->index(1)) +
                            abs(node1->index(2) - node2->index(2));
    return h;
}

double AstarPathFinder::getHeuTieBreaker(GridNodePtr node1, GridNodePtr node2)
{
    // 对角距离
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));
    double h = dx + dy + dz + (sqrt(3) - 3) * std::min(std::min(dx, dy), dz);
    double p = 1 / 40;
    h*=(1 + p);
    return h;
}

double AstarPathFinder::getHeuEuclideanTieBreaker(GridNodePtr node1, GridNodePtr node2)
{
    // 这里使用欧式距离
    double h = sqrt(
        (node1->index(0) - node2->index(0)) * (node1->index(0) - node2->index(0)) +
        (node1->index(1) - node2->index(1)) * (node1->index(1) - node2->index(1)) +
        (node1->index(2) - node2->index(2)) * (node1->index(2) - node2->index(2))
    );
    double p = 1 / 40;
    h*=(1 + p);
    return h;
}

double AstarPathFinder::getHeuManhattanTieBreaker(GridNodePtr node1, GridNodePtr node2)
{
    // 曼哈顿距离
    double h = abs(node1->index(0) - node2->index(0)) +
                            abs(node1->index(1) - node2->index(1)) +
                            abs(node1->index(2) - node2->index(2));
    double p = 1 / 40;
    h*=(1 + p);
    return h;
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */
   // 访问相邻栅格节点
   Eigen::Vector3i center = currentPtr->index;
   GridNodePtr gridPtr;
   // 先x轴方向移动
   for(int x = -1; x < 2 ; x++)
   {
        // 判断是否在地图范围内
        if(center(0) + x >= 0 && center(0) + x <= map_->mp_.map_voxel_num_.x())
            // y轴方向移动
            for(int y = -1 ; y < 2 ; y++)
            {
                // 判断是否在地图范围内
                if(center(1) + y >= 0 && center(1) + y <= map_->mp_.map_voxel_num_.y()) // z轴方向移动
                    for(int z = -1; z < 2 ; z++)
                    {
                        if(center(2) + z >= 0 && center(2) + z <= map_->mp_.map_voxel_num_.z())
                            gridPtr = GridNodeMap_[center(0) + x][center(1) + y][center(2) + z];// 如果该点为障碍物或者被访问过进入下一循环
                        if(map_->isOccupied(gridPtr->index) ||  gridPtr->id == NodeState::EXPANDED) // 曾经扩展过
                            continue;
                        else
                        {
                            // 将该点装入neighborPtrSet, 并计算edgeCostSet
                            neighborPtrSets.push_back(gridPtr);
                            // 欧式距离
                            edgeCostSets.push_back(
                                sqrt(
                                    (center(0) - gridPtr->index(0)) * (center(0) - gridPtr->index(0)) +
                                    (center(1) - gridPtr->index(1)) * (center(1) - gridPtr->index(1)) +
                                    (center(2) - gridPtr->index(2)) * (center(2) - gridPtr->index(2))
                                )
                            );
                        }
                    }
            }
    }
}

inline void AstarPathFinder::resetGridMap(GridNodePtr *** GridNodeMap, std::vector<Eigen::Vector3i> &expanded)
{
    for(auto &p:expanded)
    {
    GridNodeMap[p.x()][p.y()][p.z()]->id = NodeState::NONE;
    GridNodeMap[p.x()][p.y()][p.z()]->dir = Eigen::Vector3i::Zero();
    GridNodeMap[p.x()][p.y()][p.z()]->gScore = inf;
    GridNodeMap[p.x()][p.y()][p.z()]->fScore = inf;
    GridNodeMap[p.x()][p.y()][p.z()]->cameFrom = NULL;
    }
    expanded.clear();
}

void AstarPathFinder::AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Eigen::Vector3i start_idx = coord2gridIndex(start_pt);
    Eigen::Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx_ = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet_.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = NodeState::UNEXPANDED; 
    startPtr -> coord = start_pt;
    openSet_.insert( std::make_pair(startPtr -> fScore, startPtr) );
    //ROS_INFO("id:%d", GridNodeMap[startPtr->index[0]][startPtr->index[1]][startPtr->index[2]]->id);
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    GridNodeMap_[start_idx[0]][start_idx[1]][start_idx[2]]->id = NodeState::UNEXPANDED;
    std::vector<GridNodePtr> neighborPtrSets;
    std::vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet_.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
       // 将当前的指针指向openSet中的GridNodePtr
       currentPtr = openSet_.begin()->second;
       // 放入 close set
       GridNodeMap_[currentPtr->index[0]][currentPtr->index[1]][currentPtr->index[2]]->id = NodeState::EXPANDED;
        openSet_.erase(openSet_.begin());

        // if the current node is the goal 
        if( currentPtr->index == goalIdx_ ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr_ = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * map_->mp_.resolution_ );            
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
           neighborPtr = neighborPtrSets[i];
            if(neighborPtr -> id == NodeState::NONE){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
               neighborPtr->gScore = edgeCostSets[i] + currentPtr->gScore;
               neighborPtr->fScore = getHeu(neighborPtr, endPtr) + neighborPtr->gScore;
               // 记录前一个节点
               neighborPtr->cameFrom = currentPtr;
               // 将 🆔id设置为1
               neighborPtr->id = NodeState::UNEXPANDED;
               // 并加入OpenSet
                openSet_.insert(
                    std::make_pair(neighborPtr->fScore, neighborPtr)
                );

                continue;
            }
            else if(neighborPtr -> id == NodeState::UNEXPANDED){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
               if(neighborPtr->gScore > (edgeCostSets[i] + currentPtr->gScore))
                {
                    neighborPtr->gScore = edgeCostSets[i] + currentPtr->gScore;
                    neighborPtr->fScore = getHeu(neighborPtr, endPtr) + neighborPtr->gScore;
                    neighborPtr->cameFrom = currentPtr;
                }
                continue;
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *        
                */
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}

} // namespace CUADC

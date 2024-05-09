#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

namespace CUADC
{
#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

enum NodeState{
  NONE=0u,          // not in open set and close set
  UNEXPANDED=1u,    // in open set    1
  EXPANDED=2u       // in close set  -1
};

struct GridNode
{     
    NodeState id;        // 1--> open set, 2 --> closed set
    Eigen::Vector3d coord;    // 实际坐标
    Eigen::Vector3i dir;   // direction of expanding
    Eigen::Vector3i index;   // 三维栅格坐标
	
    double gScore, fScore;
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
		id = NodeState::NONE;
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector3i::Zero();

		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};

} // namespace CUADC
#endif

#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
// #include <Eigen/MatrixFunctions>

namespace CUADC{

class TrajectoryGen{

public:
    TrajectoryGen(){};
    ~TrajectoryGen(){};
    void initTrajectoryGen(const double acc, const double vel){acc_ = acc, vel_ = vel;}
    bool initTraj(const Eigen::MatrixXd &positions, const Eigen::Vector3d initialVel, const Eigen::Vector3d initialAcc);
    void TimeMatrix(double t, Eigen::Matrix<double, 3, 6> &time_matrix);
    bool minimumJerkTrajGen();   
    inline double timeTraj(const double dist, const double vel, const double acc);
    inline uint32_t getPieceNum(){return piece_num_;};
    inline double getPieceTime(uint32_t i){return timeAllocationVector_(i);};
    // inline Eigen::Vector3d getIntermediatePositions(uint32_t i){return intermediatePositions_.block<3, 1>(0, i);}
    inline double TotalTime(){
        double sum = 0.0;
        for(uint32_t i = 0; i < piece_num_; ++i)
            sum += timeAllocationVector_(i);
        ROS_INFO_STREAM("sum time is " << sum);
        return sum;
    }
protected:
    double acc_;
    double vel_;
    uint32_t piece_num_;
    Eigen::VectorXd time_;
    Eigen::MatrixXd waypoint_;
    Eigen::Vector3d initialPos_;
    Eigen::Vector3d initialVel_;
    Eigen::Vector3d initialAcc_;
    Eigen::Vector3d terminalPos_;
    Eigen::Vector3d terminalVel_;
    Eigen::Vector3d terminalAcc_;
    Eigen::Matrix3Xd intermediatePositions_;
    Eigen::VectorXd timeAllocationVector_;
public:
    Eigen::MatrixX3d coefficientMatrix_;
};


}
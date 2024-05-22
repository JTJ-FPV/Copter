#include "trajectory_gen/minimumJerk.h"

namespace CUADC{

bool TrajectoryGen::initTraj(const Eigen::MatrixXd &positions, const Eigen::Vector3d initialVel, const Eigen::Vector3d initialAcc)
{
    // 剔除相同点
    Eigen::Vector3d current_pt, last_pt;
    bool sign = false;
    std::vector<Eigen::Vector3d> tmp;
    for(uint32_t i = 0; i < positions.cols(); i++)
    {
        if(!sign)
        {
            last_pt = positions.block<3, 1>(0, i);
            current_pt = positions.block<3, 1>(0, i);
            tmp.push_back(current_pt);
            sign = true;
        }
        else
        {
            last_pt = current_pt;
            current_pt = positions.block<3, 1>(0, i);
            if(last_pt != current_pt)
                tmp.push_back(current_pt);
        }
    }
    // if(positions.cols() <= 1)
    //     return false;
    if(tmp.size() < 2)
    { 
        for(auto p:tmp)
            ROS_INFO_STREAM("the tmp is " << p);
        ROS_INFO_STREAM("position is " << positions);
        return false;
    }
    piece_num_ = tmp.size() - 1;
    ROS_INFO_STREAM("the piece number is " << piece_num_);
    // 路标点
    waypoint_.resize(3, tmp.size());
    for(uint32_t i = 0 ; i < tmp.size(); ++i)
    {
        waypoint_.block<3, 1>(0, i) = tmp.at(i);
    }
    ROS_INFO_STREAM("the waypoint is " << '\n' << waypoint_);
    // 计算分配时间向量
    time_.resize(piece_num_, 1);
    for(uint32_t i = 0; i < piece_num_; ++i)
    {
        double dist = (waypoint_.col(i + 1) - waypoint_.col(i)).norm();   
        ROS_INFO_STREAM("the dist is " << dist);
        time_(i) = timeTraj(dist, vel_, acc_);
        ROS_INFO_STREAM("the time is " << timeTraj(dist, vel_, acc_));
    }
    ROS_INFO_STREAM("the time vector is " << '\n' << time_);
    // 飞机当前的状态
    initialPos_ = waypoint_.col(0);
    initialVel_ = initialVel;
    initialAcc_ = initialAcc;
    ROS_INFO_STREAM("initial pos ,vel ,acc is " << initialPos_ << ", " << initialVel_ << ", " << initialAcc_);
    // 目标点的状态
    terminalPos_ = waypoint_.col(piece_num_);
    terminalVel_ = Eigen::Vector3d::Zero();
    terminalAcc_ = Eigen::Vector3d::Zero();
    // 中间点
    intermediatePositions_ = waypoint_.middleCols(1, piece_num_ - 1);
    // 时间分配向量
    timeAllocationVector_ = time_.head(piece_num_);
    // 五次多项式的轨迹参数
    coefficientMatrix_ = Eigen::MatrixXd::Zero(6 * piece_num_, 3);
    return true;
}

void TrajectoryGen::TimeMatrix(double t, Eigen::Matrix<double, 3, 6> &time_matrix)
{
    time_matrix << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
                   0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
                   0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);
}

inline double TrajectoryGen::timeTraj(const double dist, const double vel, const double acc)
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

bool TrajectoryGen::minimumJerkTrajGen()
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

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(piece_num_ * 6, piece_num_ * 6);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(piece_num_ * 6, 3);
    Eigen::Matrix<double, 3, 6> F_0, E_pieceNum;
    F_0 << 1, 0, 0, 0, 0, 0, 
           0, 1, 0, 0, 0, 0,
           0, 0, 2, 0, 0, 0;
    M.block(0, 0, 3, 6) = F_0;
    double T(timeAllocationVector_(piece_num_ - 1));
    E_pieceNum << 1 , T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5),
                  0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
                  0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
    M.block((piece_num_ - 1) * 6 + 3, (piece_num_ - 1) * 6, 3, 6) = E_pieceNum;
    b.block(0, 0, 3, 3) << initialPos_(0), initialPos_(1), initialPos_(2),
                           initialVel_(0), initialVel_(1), initialVel_(2),
                           initialAcc_(0), initialAcc_(1), initialAcc_(2);
    b.block(3 + (piece_num_ - 1) * 6, 0, 3, 3) << terminalPos_(0), terminalPos_(1), terminalPos_(2),
                                                terminalVel_(0), terminalVel_(1), terminalVel_(2),
                                                terminalAcc_(0), terminalAcc_(1), terminalAcc_(2);
    Eigen::Matrix<double, 6, 6> F_i;
    F_i << 0, 0, 0, 0, 0, 0,
          -1, 0, 0, 0, 0, 0,
          0, -1, 0, 0, 0, 0,
          0, 0, -2, 0, 0, 0,
          0, 0, 0, -6, 0, 0,
          0, 0, 0, 0, -24, 0;
    for(uint32_t i = 1; i < piece_num_; ++i)
    {
        double t_i(timeAllocationVector_(i - 1));
        Eigen::Matrix<double, 6, 6> E_i;
        Eigen::Vector3d D_i(intermediatePositions_.transpose().row(i - 1));
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
    clock_t time_stt = clock();
    // 使用PartialPivLU进行分解
    // std::cout << "use lu" <<std::endl;
    // coefficientMatrix_ = M.lu().solve(b);
    // std::cout << "Time cost = " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << std::endl;

    // coefficientMatrix_ = M.inverse();
    Eigen::PartialPivLU<Eigen::MatrixXd> plu(M);
    coefficientMatrix_ = plu.solve(b);
    if(hasNan(coefficientMatrix_))
    {
        ROS_WARN("Minimum Jerk matrix is not invertible");
        return false;
    }
    else
    {
        ROS_INFO_STREAM("PLU solve : " << coefficientMatrix_);
        ROS_INFO_STREAM("PLU Optimization time cost " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms");
        return true;
    }
    // Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(M);
    // if(!lu_decomp.isInvertible()){
    //     ROS_WARN("Minimum Jerk matrix is not invertible");
    //     // ROS_INFO_STREAM("the Matrix M is " << '\n' << M);
    //     // ROS_INFO_STREAM("the M determinant is " << M.determinant());
    //     // coefficientMatrix_ = plu.solve(b);
    //     // ROS_INFO_STREAM("PLU solve : " << coefficientMatrix_);
    //     // ROS_INFO_STREAM("PLU Optimization time cost " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms");
    //     // return true;
    // }
    // else{
    //     coefficientMatrix_ = lu_decomp.inverse() * b;
    //     ROS_INFO_STREAM("Optimization time cost " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms");
    //     return true;
    // }
    // coefficientMatrix_ = M.colPivHousederQholr().solve(b);
    // coefficientMatrix_ = M.partialPivLu().solve(b);
    // ROS_INFO_STREAM("time vector is " << timeAllocationVector_);
    // ROS_INFO_STREAM("time vector is " << time_);
}

} // namespace CUADC

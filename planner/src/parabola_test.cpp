#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <trajectory_msgs/Trajectory.h>

geometry_msgs::PoseStamped pose;
geometry_msgs::Twist velocity_b;
trajectory_msgs::Trajectory parabola;
ros::Publisher parabola_pub;
double m = 1.5, k = 0.5, g = 9.83, dt = 0.01;

double Traj_X(double t, Eigen::Vector3d V_i, Eigen::Vector3d P_w);
double Traj_Y(double t, Eigen::Vector3d V_i, Eigen::Vector3d P_w);
double Traj_Z(double t, Eigen::Vector3d V_i, Eigen::Vector3d P_w);
bool check(double X, double Y, double Z)
{
    if(isnan(X)||isnan(Y)||isnan(Z))
        return false;
    return true;
}
void gazeboCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    parabola.traj.clear();
    parabola.Header.frame_id = "map";
    // parabola.Header.stamp = ros::Time::now();
    pose.pose = msg->pose.at(2);
    velocity_b = msg->twist.at(2);
    Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    Eigen::Vector3d V_b(velocity_b.linear.x, velocity_b.linear.y, velocity_b.linear.z), V_w, P_w(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    /* 将速度从机体坐标系转换到惯性系 */
    // V_i = q * V_b;
    V_w = V_b;
    double T;
    double Z = P_w.z(); int i = 0;
    double X, Y;    geometry_msgs::PoseStamped traj_point;
    while(Z > 0.05)
    {
        X = Traj_X(i * dt, V_w, P_w);
        Y = Traj_Y(i * dt, V_w, P_w);
        Z = Traj_Z(i * dt, V_w, P_w);
        if(check(X, Y, Z))
            ROS_BREAK();
        T = i * dt;
        i++;
        traj_point.pose.position.x = X;
        traj_point.pose.position.y = Y;
        traj_point.pose.position.z = Z;
        // ROS_INFO_STREAM("the crete position is : " << traj_point.pose.position);
        parabola.traj.push_back(traj_point);
    }
    ROS_INFO_STREAM("The fly time is : " << T);
    ROS_INFO_STREAM("the position is : " << pose.pose.position);
    ROS_INFO_STREAM("the final position is : " << traj_point.pose.position);
    parabola_pub.publish(parabola);
}
/*
 *  V_w 为世界坐标系下飞机的三轴速度，P_w 为世界坐标系下飞机的坐标，世界坐标系的原点为飞机上电时的位置，方向为FLU（前左天），
 *
*/
double Traj_X(double t, Eigen::Vector3d V_w, Eigen::Vector3d P_w)
{
    double X;
    if(V_w.x() > 0)
    {
        double C3 = - m / (k * V_w.x());
        double C4 = k * P_w.x() / m -log(std::fabs(-C3));
        X = (m / k ) * log(std::fabs(t - C3)) + m * C4 / k;
    }
    else
    {
        double C12 = 1 / V_w.x();
        double C13 = m * log(std::fabs(C12)) / k + P_w.x();
        X = - m * log(std::fabs(C12 - k * t / m)) / k + C13;
    }
    return X;
}

double Traj_Y(double t, Eigen::Vector3d V_w, Eigen::Vector3d P_w)
{
    double Y;
    if(V_w.y() > 0)
    {
        double C5 = - m / (k * V_w.y());
        double C6 = k * P_w.y() / m -log(std::fabs(-C5));
        Y = (m / k ) * log(std::fabs(t - C5)) + m * C6 / k;
    }
    else
    {
        double C14 = 1 / V_w.y();
        double C15 = m * log(std::fabs(C14)) / k + P_w.y();
        Y = -m * log(std::fabs(C14 - k * t / m)) / k + C15;
    }
    return Y;
}

double Traj_Z(double t, Eigen::Vector3d V_w, Eigen::Vector3d P_w)
{
    double Z;
    double a = -sqrt(m/(k*g));
    double b = sqrt(k/(m*g));
    if(V_w.z() > 0)
    {
        double C1 = -a * atan(b * V_w.z());
        double C2 = a * log(std::fabs(cos(C1))) / b + P_w.z();
        if(t <= C1)
        {
            Z = -(a/b) * log(std::fabs(cos((t - C1) / a))) + C2;
        }
        else
        {
            double u = 1 / b;
            double Z_c1 = C2;
            double C10 = C1;
            double C11 = Z_c1 -  a * u * M_LN2;
            Z = -0.5 * a * u * (-2 * (t - C10) / a - 2 * log(exp(-2 * (t - C10) / a) + 1)) + C11;
        }
    }
    else
    {
        double u = 1 / b;
        double C7 = a * log(std::fabs((V_w.z() - u)/V_w.z() + u)) * 0.5;
        if(-V_w.z() <= u)
        {
            double C9 = 0.5 * a * u * (2 * C7 / a - 2 * log(exp(2 * C7 / a) + 1)) + P_w.z();
            Z = -0.5 * a * u * (2 * (C7 - t) / a - 2 * log(exp(2 * (C7 - t) / a) + 1)) + C9;
        }
        else if(-V_w.z() > u)
        {
            double C8 = 0.5 * a * u * (2 * C7 / a - 2 * log(std::fabs(exp(2 * C7 / a) - 1))) + P_w.z();
            Z = -0.5 * a * u * (2 * (C7 - t) / a - 2 * log(std::fabs(exp(2 * (C7 - t) / a) - 1))) + C8;
        }
    }
    return Z;
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "parabola_test");
    setlocale(LC_ALL, "");
    ros::NodeHandle nh;
    ros::Subscriber sim_gazebo_pose_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, gazeboCallback, ros::TransportHints().tcpNoDelay());
    parabola_pub = nh.advertise<trajectory_msgs::Trajectory>("/predict/parabola", 100);
    ros::Rate rate(60);
    rate.sleep();
    ros::spin();
    return 0;
}

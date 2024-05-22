#pragma once
#include <ros/ros.h>
#include <string>

namespace CUADC
{
class VisualizerConfig
{
public:
    struct Robot
    {
        std::string Publish_RobotModelTopic;
        std::string RobotModel;
        std::string RobotModel_ns;
        int32_t RobotModel_id;
        std::string RobotModel_frame;
        double Scale_x, Scale_y, Scale_z;
    };
    

    struct Frame
    {
        std::string OriginFrame;
        std::string VehicleFrame;
        bool useFPV;
    };
    
    struct WayPoint
    {
        std::string Subscribe_WayPointTopic;
        std::string Publish_WayPointTopic;
        std::string WayPoint_ns;
        int32_t WayPoint_id;
        std::string WayPoint_frame;
        double Scale_x, Scale_y, Scale_z;
        float Color_r, Color_g, Color_b, Color_a;
        int32_t Contain;
    };
    
    struct Trajectory
    {
        std::string Subscribe_TrajectoryTopic;
        std::string Publish_TrajectoryTopic;
        std::string Trajectory_ns;
        int32_t Trajectory_id;
        std::string Trajectory_frame;
        double Scale_x;
        float Color_r, Color_g, Color_b, Color_a;
        bool History_Trajectory_Marker;
        int32_t Contain;
    };

    struct PredictTrajectory
    {
        std::string Subscribe_PredictTrajectoryTopic;
        std::string Publish_PredictTrajectoryTopic;
        std::string PredictTrajectory_ns;
        int32_t PredictTrajectory_id;
        std::string PredictTrajectory_frame;
        double Scale_x;
        float Color_r, Color_g, Color_b, Color_a;
        int32_t Contain;
    };
    

    struct HistoryTraj
    {
        std::string HistoryTrajectoryTopic;
        std::string HistoryTrajectory_frame;
        int32_t Contain;
    };
    

    struct VehiclePose
    {
        bool use_simulation;
        bool use_custom_pose_estimate;
        std::string gazebo_pose_topic;
        std::string px4_pose_topic;
        std::string px4_custom_pose_estimate_topic;
    };
    
    Robot robot;
    Frame frame;
    WayPoint waypoint;
    Trajectory trajectory;
    PredictTrajectory predict_traj;
    HistoryTraj history_traj;
    VehiclePose vehicle_pose; 
    int Rate;
    VisualizerConfig(){};
    ~VisualizerConfig(){};
    void getParamters(const ros::NodeHandle &nh);
private:
    template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};

void VisualizerConfig::getParamters(const ros::NodeHandle &nh)
{
    nh.getParam("Robot/Publish_RobotModelTopic", robot.Publish_RobotModelTopic);
    nh.getParam("Robot/RobotModel", robot.RobotModel);
    nh.getParam("Robot/RobotModel_ns", robot.RobotModel_ns);
    nh.getParam("Robot/RobotModel_id", robot.RobotModel_id);
    nh.getParam("Robot/RobotModel_frame", robot.RobotModel_frame);
    nh.getParam("Robot/Scale_x", robot.Scale_x);
    nh.getParam("Robot/Scale_y", robot.Scale_y);
    nh.getParam("Robot/Scale_z", robot.Scale_z);

    nh.getParam("Frame/OriginFrame", frame.OriginFrame);
    nh.getParam("Frame/VehicleFrame", frame.VehicleFrame);
    nh.getParam("Frame/useFPV", frame.useFPV);

    nh.getParam("WayPoint/Subscribe_WayPointTopic", waypoint.Subscribe_WayPointTopic);
    nh.getParam("WayPoint/Publish_WayPointTopic", waypoint.Publish_WayPointTopic);
    nh.getParam("WayPoint/WayPoint_ns", waypoint.WayPoint_ns);
    nh.getParam("WayPoint/WayPoint_id", waypoint.WayPoint_id);
    nh.getParam("WayPoint/WayPoint_frame", waypoint.WayPoint_frame);
    nh.getParam("WayPoint/Scale_x", waypoint.Scale_x);
    nh.getParam("WayPoint/Scale_y", waypoint.Scale_y);
    nh.getParam("WayPoint/Scale_z", waypoint.Scale_z);
    nh.getParam("WayPoint/Color_r", waypoint.Color_r);
    nh.getParam("WayPoint/Color_g", waypoint.Color_g);
    nh.getParam("WayPoint/Color_b", waypoint.Color_b);
    nh.getParam("WayPoint/Color_a", waypoint.Color_a);
    nh.getParam("WayPoint/Contain", waypoint.Contain);

    nh.getParam("Trajectory/Subscribe_TrajectoryTopic", trajectory.Subscribe_TrajectoryTopic);
    nh.getParam("Trajectory/Publish_TrajectoryTopic", trajectory.Publish_TrajectoryTopic);
    nh.getParam("Trajectory/Trajectory_ns", trajectory.Trajectory_ns);
    nh.getParam("Trajectory/Trajectory_id", trajectory.Trajectory_id);
    nh.getParam("Trajectory/Trajectory_frame", trajectory.Trajectory_frame);
    nh.getParam("Trajectory/Scale_x", trajectory.Scale_x);
    nh.getParam("Trajectory/Color_r", trajectory.Color_r);
    nh.getParam("Trajectory/Color_g", trajectory.Color_g);
    nh.getParam("Trajectory/Color_b", trajectory.Color_b);
    nh.getParam("Trajectory/Color_a", trajectory.Color_a);
    nh.getParam("Trajectory/History_Trajectory_Marker", trajectory.History_Trajectory_Marker);
    nh.getParam("Trajectory/Contain", trajectory.Contain);

    nh.getParam("PredictTrajectory/Subscribe_PredictTrajectoryTopic", predict_traj.Subscribe_PredictTrajectoryTopic);
    nh.getParam("PredictTrajectory/Publish_PredictTrajectoryTopic", predict_traj.Publish_PredictTrajectoryTopic);
    nh.getParam("PredictTrajectory/PredictTrajectory_ns", predict_traj.PredictTrajectory_ns);
    nh.getParam("PredictTrajectory/PredictTrajectory_id", predict_traj.PredictTrajectory_id);
    nh.getParam("PredictTrajectory/PredictTrajectory_frame", predict_traj.PredictTrajectory_frame);
    nh.getParam("PredictTrajectory/Scale_x", predict_traj.Scale_x);
    nh.getParam("PredictTrajectory/Color_r", predict_traj.Color_r);
    nh.getParam("PredictTrajectory/Color_g", predict_traj.Color_g);
    nh.getParam("PredictTrajectory/Color_b", predict_traj.Color_b);
    nh.getParam("PredictTrajectory/Color_a", predict_traj.Color_a);
    nh.getParam("PredictTrajectory/Contain", predict_traj.Contain);

    nh.getParam("HistoryTraj/HistoryTrajectoryTopic", history_traj.HistoryTrajectoryTopic);
    nh.getParam("HistoryTraj/HistoryTrajectory_frame", history_traj.HistoryTrajectory_frame);
    nh.getParam("HistoryTraj/Contain", history_traj.Contain);

    nh.getParam("VehiclePose/use_simulation", vehicle_pose.use_simulation);
    nh.getParam("VehiclePose/use_custom_pose_estimate", vehicle_pose.use_custom_pose_estimate);
    nh.getParam("VehiclePose/gazebo_pose_topic", vehicle_pose.gazebo_pose_topic);
    nh.getParam("VehiclePose/px4_pose_topic", vehicle_pose.px4_pose_topic);
    nh.getParam("VehiclePose/px4_custom_pose_estimate_topic", vehicle_pose.px4_custom_pose_estimate_topic);

    nh.getParam("Rate", Rate);
}

} // CUADC



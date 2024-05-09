#pragma once
#include "AcroParamters.hpp"
#include <string>
#include <vector>

namespace CUADC{

class PlannerConfig
{
public:
    struct VehicleParamters
    {
        double m, k;
        double Max_a, Max_v;
        double g, dt, Min_v_x;
    };
    

    struct MultiCopterPlannerTrajectory
    {
        bool MultiCopterPlanner;
        std::string Publish_PlannerTopic;
        std::string PlannerTrajectory_frame;
        bool use_MinimumJerk;
        std::string PublishMinimumJerk_WayPointTopic;
        bool MultiCopterControl;
        std::string Subscribe_TargetPoint_World_FLU_Topic;
        int VisionControl;
        std::vector<double> FirstPoint;
        std::vector<double> MinimumJerk_WayPoint, TerminalVelocity, TerminalAccerlatation;
        bool use_MpcPlanner;
        std::vector<double> TargetPoint;
    };
    
    struct FixedWingPlannerTrajectory
    {
        bool use_MpcPlanner;
        std::string AcroParamters_Path;
        std::string Publish_PlannerTopic;
        std::string PlannerTrajectory_frame;
        std::vector<double> TargetPoint;
    };
    

    struct ParabolaTrajectory
    {
        std::string Publish_ParabolaTrajectoryTopic;
        std::string ParabolaTrajectory_frame;
        double Parabola_terminal_high;
    };
    
    struct Control
    {
        std::string Control_topic;
        int Control_Rate;
    };
    

    struct VehiclePose
    {
        std::string px4_state_topic;
        bool use_simulation_pose;
        std::string gazebo_pose_topic;
        std::string px4_odom_topic;
        std::string vehicle_imu_topic;
    };

    VehicleParamters vehicle_paramters;
    MultiCopterPlannerTrajectory planner_trajectory;
    FixedWingPlannerTrajectory fixedwing_planner;
    AcroParamters acro_paramters;
    ParabolaTrajectory parabola_trajectory;
    Control control;
    VehiclePose vehicle_pose;
    int Rate;
    PlannerConfig(){};
    void getParam(const ros::NodeHandle &nh);
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

void PlannerConfig::getParam(const ros::NodeHandle &nh)
{
    read_essential_param(nh, "VehicleParamters/m", vehicle_paramters.m);
    read_essential_param(nh, "VehicleParamters/k", vehicle_paramters.k);
    read_essential_param(nh, "VehicleParamters/Max_a", vehicle_paramters.Max_a);
    read_essential_param(nh, "VehicleParamters/Max_v", vehicle_paramters.Max_v);
    read_essential_param(nh, "VehicleParamters/g", vehicle_paramters.g);
    read_essential_param(nh, "VehicleParamters/dt", vehicle_paramters.dt);
    read_essential_param(nh, "VehicleParamters/Min_v_x", vehicle_paramters.Min_v_x);

    read_essential_param(nh, "MultiCopterPlannerTrajectory/MultiCopterPlanner", planner_trajectory.MultiCopterPlanner);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/Publish_PlannerTopic", planner_trajectory.Publish_PlannerTopic);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/PlannerTrajectory_frame", planner_trajectory.PlannerTrajectory_frame);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/use_MinimumJerk", planner_trajectory.use_MinimumJerk);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/PublishMinimumJerk_WayPointTopic", planner_trajectory.PublishMinimumJerk_WayPointTopic);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/MultiCopterControl", planner_trajectory.MultiCopterControl);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/Subscribe_TargetPoint_World_FLU_Topic", planner_trajectory.Subscribe_TargetPoint_World_FLU_Topic);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/VisionControl", planner_trajectory.VisionControl);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/FirstPoint", planner_trajectory.FirstPoint);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/MinimumJerk_WayPoint", planner_trajectory.MinimumJerk_WayPoint);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/TerminalVelocity", planner_trajectory.TerminalVelocity);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/TerminalAccerlatation", planner_trajectory.TerminalAccerlatation);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/use_MpcPlanner", planner_trajectory.use_MpcPlanner);
    read_essential_param(nh, "MultiCopterPlannerTrajectory/TargetPoint", planner_trajectory.TargetPoint);

    read_essential_param(nh, "FixedWingPlannerTrajectory/use_MpcPlanner", fixedwing_planner.use_MpcPlanner);
    read_essential_param(nh, "FixedWingPlannerTrajectory/AcroParamters_Path", fixedwing_planner.AcroParamters_Path);
    read_essential_param(nh, "FixedWingPlannerTrajectory/Publish_PlannerTopic", fixedwing_planner.Publish_PlannerTopic);
    read_essential_param(nh, "FixedWingPlannerTrajectory/PlannerTrajectory_frame", fixedwing_planner.PlannerTrajectory_frame);
    read_essential_param(nh, "FixedWingPlannerTrajectory/TargetPoint", fixedwing_planner.TargetPoint);

    read_essential_param(nh, "ParabolaTrajectory/Publish_ParabolaTrajectoryTopic", parabola_trajectory.Publish_ParabolaTrajectoryTopic);
    read_essential_param(nh, "ParabolaTrajectory/ParabolaTrajectory_frame", parabola_trajectory.ParabolaTrajectory_frame);
    read_essential_param(nh, "ParabolaTrajectory/Parabola_terminal_high", parabola_trajectory.Parabola_terminal_high);

    read_essential_param(nh, "Control/Control_topic", control.Control_topic);
    read_essential_param(nh, "Control/Control_Rate", control.Control_Rate);

    read_essential_param(nh, "VehiclePose/px4_state_topic", vehicle_pose.px4_state_topic);
    read_essential_param(nh, "VehiclePose/use_simulation_pose", vehicle_pose.use_simulation_pose);
    read_essential_param(nh, "VehiclePose/gazebo_pose_topic", vehicle_pose.gazebo_pose_topic);
    read_essential_param(nh, "VehiclePose/px4_odom_topic", vehicle_pose.px4_odom_topic);
    read_essential_param(nh, "VehiclePose/vehicle_imu_topic", vehicle_pose.vehicle_imu_topic);

    read_essential_param(nh,"Rate", Rate);

    // read Acro Paramters
    if(fixedwing_planner.use_MpcPlanner)
    {
        ROS_ASSERT(acro_paramters.readParamters(fixedwing_planner.AcroParamters_Path));
        acro_paramters.ceresCurveFitting();
    }
}

}
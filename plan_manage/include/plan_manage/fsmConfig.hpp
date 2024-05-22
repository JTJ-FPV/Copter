#pragma once

#include <ros/ros.h>
#include <string>

namespace CUADC{

class RePlanConfig
{
public:
    struct MultiCopter
    {
        double max_vel;
        double max_acc;
        double hover_height;
    };
    

    struct Replan
    {
        double target_replan_thresh;
        double start_replan_thresh;
        double replan_time;
        double dt;
    };
    
    struct Publish_Replan
    {
        std::string Publish_AstarPath;
        std::string Publish_Replan_AstarPath;
        std::string Publish_RdpPath;
        std::string Publish_MinimJerkPath;
        std::string Publish_Control;
    };
    
    struct Subscribe_Replan
    {
        std::string Subscribe_State;
        std::string Subscribe_Odometry;
        std::string Subscribe_Imu;
    };
    
    MultiCopter copter;
    Replan replan;
    Publish_Replan publish_replan;
    Subscribe_Replan subscribe_replan;

    RePlanConfig(){};
    ~RePlanConfig(){};
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

void RePlanConfig::getParamters(const ros::NodeHandle &nh)
{
    read_essential_param(nh, "MultiCopter/max_vel", copter.max_vel);
    read_essential_param(nh, "MultiCopter/max_acc", copter.max_acc);
    read_essential_param(nh, "MultiCopter/hover_height", copter.hover_height);

    read_essential_param(nh, "Replan/target_replan_thresh", replan.target_replan_thresh);
    read_essential_param(nh, "Replan/start_replan_thresh", replan.start_replan_thresh);
    read_essential_param(nh, "Replan/replan_time", replan.replan_time);
    read_essential_param(nh, "Replan/dt", replan.dt);

    read_essential_param(nh, "Publish_Replan/Publish_AstarPath", publish_replan.Publish_AstarPath);
    read_essential_param(nh, "Publish_Replan/Publish_Replan_AstarPath", publish_replan.Publish_Replan_AstarPath);
    read_essential_param(nh, "Publish_Replan/Publish_RdpPath", publish_replan.Publish_RdpPath);
    read_essential_param(nh, "Publish_Replan/Publish_MinimJerkPath", publish_replan.Publish_MinimJerkPath);
    read_essential_param(nh, "Publish_Replan/Publish_Control", publish_replan.Publish_Control);
    
    read_essential_param(nh, "Subscribe_Replan/Subscribe_State", subscribe_replan.Subscribe_State);
    read_essential_param(nh, "Subscribe_Replan/Subscribe_Odometry", subscribe_replan.Subscribe_Odometry);
    read_essential_param(nh, "Subscribe_Replan/Subscribe_Imu", subscribe_replan.Subscribe_Imu);
}


}
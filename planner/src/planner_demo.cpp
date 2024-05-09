#include <ros/ros.h>
#include "Planner.hpp"
#include <mavros_msgs/PositionTarget.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_test");
    ros::NodeHandle nh("~");
    ros::Rate rate(20);
    CUADC::Planner planner(nh);
    // 多线程
    ros::MultiThreadedSpinner spinner;
    // mavros_msgs::PositionTarget position_home, position_hold;
    // position_home.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // position_home.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
    //                           mavros_msgs::PositionTarget::IGNORE_VY |
    //                           mavros_msgs::PositionTarget::IGNORE_VZ |
    //                           mavros_msgs::PositionTarget::IGNORE_AFX |
    //                           mavros_msgs::PositionTarget::IGNORE_AFY |
    //                           mavros_msgs::PositionTarget::IGNORE_AFZ |
    //                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    // position_hold.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // position_hold.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
    //                           mavros_msgs::PositionTarget::IGNORE_VY |
    //                           mavros_msgs::PositionTarget::IGNORE_VZ |
    //                           mavros_msgs::PositionTarget::IGNORE_AFX |
    //                           mavros_msgs::PositionTarget::IGNORE_AFY |
    //                           mavros_msgs::PositionTarget::IGNORE_AFZ |
    //                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    
    // while(planner.minijerk_trajectory.empty())
    // {
    //     ros::spinOnce();
    // }
    // position_home = planner.minijerk_trajectory.front();
    // position_hold = planner.minijerk_trajectory.back();
    // ROS_INFO("the planner demo");
    // for(int i = 100; ros::ok() && i > 0; --i)
    // {
    //     planner.controlPub.publish(position_home);
    //     rate.sleep();
    // }

    // ROS_INFO("waiting for offboard mode");
    // while(ros::ok()){
    //     planner.controlPub.publish(position_home);
    //     ros::spinOnce();
    //     rate.sleep();
    //     if(planner.current_state.mode == "OFFBOARD" && planner.current_state.armed) break;
    // }

    // int step = 0;
    // uint32_t count = 0;
    // while(ros::ok())
    // {
    //     switch ((step))
    //     {
    //         case 0:
    //         {
    //             planner.controlPub.publish(position_home);
    //             step = 1;
    //             break;
    //         }
    //         case 1:
    //         {
    //             if(count < planner.minijerk_trajectory.size())
    //             {
    //                 planner.controlPub.publish(planner.minijerk_trajectory.at(count++));
    //             }
    //             else
    //             {
    //                 planner.controlPub.publish(position_hold);
    //             }
    //         }
    //     }
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // ros::spin();
    spinner.spin();
    return 0;
}

#include <ros/ros.h>
#include "plan_manage/plan_fsm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan_manage");
    ros::NodeHandle nh("~");
    CUADC::PlanFSM plan_fsm(nh);
    // ros::MultiThreadedSpinner spinner(10);
    // spinner.spin();
    ros::spin();
    return 0;
}

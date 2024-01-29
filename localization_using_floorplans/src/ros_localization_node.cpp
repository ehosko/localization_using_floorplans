#include "../include/localization/simple_localizer.h"

#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_localization_node");

    ros::NodeHandle nh("");

    SimpleLocalizer simple_localizer(nh);
    simple_localizer.setupFromParam();
    //simple_localizer.localize();

    ros::spin();

    return 0;
}
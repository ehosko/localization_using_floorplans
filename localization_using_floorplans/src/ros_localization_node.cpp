#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_localization_node");

    ros::NodeHandle nh;

    ros::spin();

    return 0;
}
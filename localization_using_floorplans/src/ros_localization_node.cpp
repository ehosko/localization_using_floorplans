#include "../include/localization/simple_localizer.h"
#include "../include/referenceTrajectory/floorplan_graph.h"

#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_localization_node");

    ros::NodeHandle nh("");

    FloorplanGraph floorplan_graph;
    floorplan_graph.initFloorplanGraph(nh);

    // SimpleLocalizer simple_localizer(nh);
    // simple_localizer.setupFromParam();
    //simple_localizer.localize();

    ros::spin();

    //simple_localizer.transformationFile_.close();
    return 0;
}
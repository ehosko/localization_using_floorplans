#include "../include/localization/simple_localizer.h"
#include "../include/referenceTrajectory/floorplan_graph.h"

#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_localization_node");

    ros::NodeHandle nh("");

    bool use_floorplan;
    nh.getParam("floorplan_node/use_floorplan_loc", use_floorplan);

    FloorplanGraph floorplan_graph;
    floorplan_graph.initFloorplanGraph(nh);

    SimpleLocalizer simple_localizer(nh);
    if (use_floorplan)
    {
        simple_localizer.setupFromParam();
    }

    ros::spin();

    floorplan_graph.path_file_.close();
    floorplan_graph.occupancy_file_.close();
    floorplan_graph.occupancy_opt_file_.close();

    if (use_floorplan)
        simple_localizer.transformationFile_.close();

        
    return 0;
}
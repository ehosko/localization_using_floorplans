#ifndef SIMPLE_LOCALIZER_H
#define SIMPLE_LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/registration/gicp.h>

#include "cloudGenerator/source_generator.h"
#include "cloudGenerator/target_generator.h"

class SimpleLocalizer
{
    public:
        SimpleLocalizer(); // Constructor
        SimpleLocalizer(const ::ros::NodeHandle& nh) : nh_(nh) {} // Constructor
        ~SimpleLocalizer(); // Destructor

        void localize();

        void setupFromParam();

        int computeTransformationGICP(pcl::PointCloud<pcl::PointXYZ> sourceCloud, 
                                    pcl::PointCloud<pcl::PointXYZ> targetCloud,
                                    Eigen::Matrix4f& transformationMatrix);

        void odomCallback(const nav_msgs::Odometry& msg);

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    protected:
        ::ros::NodeHandle nh_;

        SourceGenerator sourceGenerator_;
        TargetGenerator targetGenerator_;


};


#endif // SIMPLE_LOCALIZER_H
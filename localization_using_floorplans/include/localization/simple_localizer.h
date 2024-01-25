#ifndef SIMPLE_LOCALIZER_H
#define SIMPLE_LOCALIZER_H

#include <ros/ros.h>
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

    protected:
        ::ros::NodeHandle nh_;

        SourceGenerator sourceGenerator_;
        TargetGenerator targetGenerator_;


};


#endif // SIMPLE_LOCALIZER_H
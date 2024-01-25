#ifndef SOURCE_GENERATOR_H
#define SOURCE_GENERATOR_H

#include <pcl/point_cloud.h>
#include <pcl/filters/project_inliers.h>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>

class SourceGenerator
{
    public:
        SourceGenerator(); // Constructor
        ~SourceGenerator(); // Destructor

        void initSourceGenerator(ros::NodeHandle& nh);

        void ProjectOnFloor(pcl::PointCloud<pcl::PointXYZ> depthCloud,Eigen::Quaterniond q);
        void RemoveOutliers();
        void RemoveOutliersPlaneInformation();
        void RemoveOutliersGICP();

        pcl::PointCloud<pcl::PointXYZ> _sourceCloud;

    private:

};

#endif // SOURCE_GENERATOR_H
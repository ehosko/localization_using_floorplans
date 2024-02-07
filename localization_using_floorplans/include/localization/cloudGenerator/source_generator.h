#ifndef SOURCE_GENERATOR_H
#define SOURCE_GENERATOR_H

#include <pcl/point_cloud.h>
#include <pcl/filters/project_inliers.h>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/console.h>

#include "../utils/process_point_cloud.h"

class SourceGenerator
{
    public:
        SourceGenerator(); // Constructor
        ~SourceGenerator(); // Destructor

        void initSourceGenerator(ros::NodeHandle& nh);

        void generateSourceCloud(pcl::PointCloud<pcl::PointXYZ> depthCloud, Eigen::Quaterniond q);

        pcl::PointCloud<pcl::PointXYZ> _sourceCloud;

        void cleanSourceCloud();

    private:

        ros::Subscriber transformSub_;

        void transformCallback(const geometry_msgs::TransformStamped& transform_msg);

        void ProjectOnFloor(pcl::PointCloud<pcl::PointXYZ> depthCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, Eigen::Quaterniond q);
        void RemoveOutliers();
        void RemoveOutliersPlaneInformation();
        void RemoveOutliersGICP();

        Eigen::Transform<double, 3, Eigen::Affine> transform_;
        Eigen::Transform<double, 3, Eigen::Affine> T_B_C_;
};

#endif // SOURCE_GENERATOR_H
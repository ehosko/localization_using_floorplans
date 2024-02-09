#ifndef SOURCE_GENERATOR_H
#define SOURCE_GENERATOR_H

#include <pcl/point_cloud.h>
#include <pcl/filters/project_inliers.h>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>

#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "../utils/process_point_cloud.h"

class SourceGenerator
{
    public:
        SourceGenerator(); // Constructor
        ~SourceGenerator(); // Destructor

        void initSourceGenerator(ros::NodeHandle& nh);

        void generateSourceCloud(pcl::PointCloud<pcl::PointXYZ> depthCloud, Eigen::Quaterniond q,const ros::Time& timestamp);

        void projectPosOnFloor(Eigen::Vector3d position, Eigen::Quaterniond orientation);

        pcl::PointCloud<pcl::PointXYZ> _sourceCloud;
        std::vector<sensor_msgs::PointCloud2ConstPtr> depthCloud_msg_;

        void cleanSourceCloud();

    private:

        ros::Subscriber transformSub_;

        void transformCallback(const geometry_msgs::TransformStamped& transform_msg);

        void TransformPoints(pcl::PointCloud<pcl::PointXYZ> depthCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloudTransform,Eigen::Quaterniond q,const ros::Time& timestamp);
        void ProjectOnFloor(pcl::PointCloud<pcl::PointXYZ> depthCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, Eigen::Quaterniond q);

        Eigen::Transform<double, 3, Eigen::Affine> transform_;
        Eigen::Transform<double, 3, Eigen::Affine> T_B_C_;

        std::vector<geometry_msgs::TransformStamped> transform_msg_;


        int timestamp_tolerance_ns_ = 1000000;
};

#endif // SOURCE_GENERATOR_H
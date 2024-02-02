#ifndef SIMPLE_LOCALIZER_H
#define SIMPLE_LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>

#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>


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
                                    Eigen::Matrix4d& transformationMatrix);

        void odomCallback(const nav_msgs::Odometry& msg);

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

        void publishTransformation();

        std::ofstream transformationFile_;

    protected:
        ::ros::NodeHandle nh_;
        ::ros::Subscriber odomSub_;
        ::ros::Subscriber cloudSub_;
        tf2_ros::TransformBroadcaster broadcaster;

        SourceGenerator sourceGenerator_;
        TargetGenerator targetGenerator_;

        Eigen::Vector3d position_;
        Eigen::Quaterniond orientation_;
        
        double d_l_;
        double theta_l_;

        pcl::PointCloud<pcl::PointXYZ> depthCloud_;

        Eigen::Matrix4d transformationMatrix_;

    
};


#endif // SIMPLE_LOCALIZER_H
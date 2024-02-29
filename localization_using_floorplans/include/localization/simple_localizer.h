#ifndef SIMPLE_LOCALIZER_H
#define SIMPLE_LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

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
        SimpleLocalizer(const ::ros::NodeHandle& nh) : nh_(nh), tfListener_(tfBuffer_) {} // Constructor
        ~SimpleLocalizer(); // Destructor

        void setupFromParam();

        int computeTransformationGICP(pcl::PointCloud<pcl::PointXYZ> sourceCloud, 
                                    pcl::PointCloud<pcl::PointXYZ> targetCloud,
                                    Eigen::Matrix4d& transformationMatrix);

        void odomCallback(const nav_msgs::Odometry& msg);

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

        void syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const nav_msgs::OdometryConstPtr& odom_msg);

        void publishTransformation(Eigen::Vector3d position, Eigen::Quaterniond orientation);

        std::ofstream transformationFile_;

    protected:
        ::ros::NodeHandle nh_;
        ::ros::Subscriber odomSub_;
        ::ros::Subscriber cloudSub_;
        
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;
        tf2_ros::TransformBroadcaster broadcaster_;

        SourceGenerator sourceGenerator_;
        TargetGenerator targetGenerator_;

        Eigen::Vector3d position_;
        Eigen::Quaterniond orientation_;

        Eigen::Vector3d floorplan_position_;
        //Eigen::Quaterniond floorplan_orientation_;
        
        double d_l_; // distance threshold before updating estimate
        double theta_l_; // angle threshold before updating estimate

        double transformationThreshold_; // threshold to determine if transformation is valid

        pcl::PointCloud<pcl::PointXYZ> depthCloud_;

        Eigen::Matrix4d transformationMatrix_;

    
};


#endif // SIMPLE_LOCALIZER_H
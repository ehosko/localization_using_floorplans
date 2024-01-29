#ifndef TARGET_GENERATOR_H
#define TARGET_GENERATOR_H

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

#include "../utils/simple_ray_caster.h"

class TargetGenerator
{
    public:
        TargetGenerator(); // Constructor
        TargetGenerator(std::string image_path) : image_(cv::imread(image_path)) {} // Constructor
        ~TargetGenerator(); // Destructor

        void initTargetGenerator(ros::NodeHandle& nh);

        void generateTargetCloud(Eigen::Vector3d position, Eigen::Quaterniond orientation);
        std::vector<std::vector<cv::Point>> getContours(){return contours_;};
        std::vector<cv::Point> getSegments();
        bool aStar(cv::Point, cv::Point, std::vector<cv::Point>&);

        //Public Member
        pcl::PointCloud<pcl::PointXYZ> _targetCloud;

    private:
        void computeContours();
        void divideContours();
        void filterAccesiblePoints(const std::vector<cv::Point>&,Eigen::Vector3d position, Eigen::Quaterniond orientation);
        double getDistance(cv::Point, cv::Point);
        

        //Parameters
        cv::Mat image_; //OpenCV Image

        int image_width_ = 0; //Image width
        int image_height_ = 0; //Image height

        int experiment_width_ = 0; //Floorplan width
        int experiment_height_ = 0; //Floorplan height

        std::vector<std::vector<cv::Point>> contours_; //Contours
        std::vector<cv::Point> candidate_points_; //Contours
        int l_max_ = 20; //Max length of segment
        //PCL Point Cloud

        SimpleRayCaster raycaster_;

};

#endif // TARGET_GENERATOR_H
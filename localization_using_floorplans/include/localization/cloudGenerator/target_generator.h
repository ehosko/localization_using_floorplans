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
        TargetGenerator(std::string image_path) : image(cv::imread(image_path)) {} // Constructor
        ~TargetGenerator(); // Destructor

        void initTargetGenerator(ros::NodeHandle& nh);

        void generateTargetCloud();
        std::vector<std::vector<cv::Point>> getContours(){return contours;};
        std::vector<cv::Point> getSegments();
        bool aStar(cv::Point, cv::Point, std::vector<cv::Point>&);

        //Public Member
        pcl::PointCloud<pcl::PointXYZ> _targetCloud;

    private:
        void computeContours();
        std::vector<cv::Point> divideContours();
        void filterAccesiblePoints(const std::vector<cv::Point>&,Eigen::Vector2d position, Eigen::Quaterniond orientation);
        double getDistance(cv::Point, cv::Point);
        

        //Parameters
        cv::Mat image; //OpenCV Image
        std::vector<std::vector<cv::Point>> contours; //Contours
        int l_max = 20; //Max length of segment
        //PCL Point Cloud

        SimpleRayCaster raycaster_;

};

#endif // TARGET_GENERATOR_H
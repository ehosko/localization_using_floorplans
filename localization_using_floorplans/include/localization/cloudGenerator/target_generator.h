#ifndef TARGET_GENERATOR_H
#define TARGET_GENERATOR_H

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

#include "../utils/simple_ray_caster.h"
#include "../utils/process_point_cloud.h"
#include "../utils/aStar.h"

class TargetGenerator
{
    public:
        TargetGenerator(); // Constructor
        TargetGenerator(std::string image_path) : image_(cv::imread(image_path)) {} // Constructor
        ~TargetGenerator(); // Destructor

        TargetGenerator& operator=(const TargetGenerator& other); // Copy assignment

        void initTargetGenerator(ros::NodeHandle& nh);

        void generateTargetCloud(Eigen::Vector3d position, Eigen::Quaterniond orientation);
        std::vector<std::vector<cv::Point>> getContours(){return contours_;};
        std::vector<cv::Point2f> getSegments();
      

        void cleanTargetCloud();

        //Public Member
        pcl::PointCloud<pcl::PointXYZ> _targetCloud;

    private:
        void computeContours();
        void divideContours();
        void filterAccesiblePoints(std::vector<cv::Point2f>&,const std::vector<cv::Point2f>&,Eigen::Vector3d position, Eigen::Quaterniond orientation);
        double getDistance(cv::Point2f, cv::Point2f);
        

        //Parameters
        cv::Mat image_; //OpenCV Image

        int image_width_ = 0; //Image width
        int image_height_ = 0; //Image height

        int experiment_width_ = 0; //Floorplan width
        int experiment_height_ = 0; //Floorplan height

        std::vector<std::vector<cv::Point>> contours_; //Contours
        std::vector<cv::Point2f> segments_; //Segments

        std::vector<cv::Point2f> candidate_points_; //Candidate points
        
        int l_max_ = 2; //Max length of segment
        //PCL Point Cloud

        double epsilon_ = 0.01; // Espilon to consider node reached

        double resolution_ = 0.1; //Resolution of the floorplan

        SimpleRayCaster raycaster_;

        double x_min_ = 0;
        double y_max_ = 0;
};

#endif // TARGET_GENERATOR_H
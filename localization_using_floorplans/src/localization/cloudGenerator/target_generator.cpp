#include "../../../include/localization/cloudGenerator/target_generator.h"
#include <stdlib.h>


TargetGenerator::TargetGenerator()
{
    // Constructor
}

TargetGenerator::~TargetGenerator()
{
    // Destructor
}

TargetGenerator& TargetGenerator::operator=(const TargetGenerator& other)
{
    // Copy assignment
    if(this != &other)
    {
        image_ = other.image_;
    }
    return *this;
}

void TargetGenerator::initTargetGenerator(ros::NodeHandle& nh)
{
    // Initialize TargetGenerator
    std::cout << "Initializing TargetGenerator..." << std::endl;
    nh.getParam("floorplan_node/l_max", l_max_);
    nh.getParam("floorplan_node/floorplan_width", experiment_width_);
    nh.getParam("floorplan_node/floorplan_height", experiment_height_);

    nh.getParam("floorplan_node/voxel_size", resolution_);

    // image_height_ = image_.rows;
    // image_width_ = image_.cols;

    image_height_ = experiment_height_ / resolution_;
    image_width_ = experiment_width_ / resolution_;

    std::cout << "Image size: " << image_width_ << "x" << image_height_ << std::endl;

    computeContours();
    divideContours();

    // Transform Points
    for(int i = 0; i < segments_.size(); i++)
    {
        segments_[i].x = (segments_[i].x * ((float)experiment_width_/image_width_) - 0.5f*experiment_width_);
        segments_[i].y = -(segments_[i].y * ((float)experiment_height_/image_height_) - 0.5f*experiment_height_);

        std::cout << "Candidate point: " << segments_[i] << std::endl;
    }

    // Initialize SimpleRayCaster
    raycaster_ = SimpleRayCaster();
    raycaster_.initSimpleRayCaster(nh);
}

void TargetGenerator::generateTargetCloud(Eigen::Vector3d position, Eigen::Quaterniond orientation)
{
    // Generate target cloud
    std::cout << "Generating target cloud..." << std::endl;
  
    
    std::cout << "Filter Points..." << std::endl;
    //std::vector<cv::Point2f> candidates = segments_;
    std::vector<cv::Point2f> accessiblePoints;
    filterAccesiblePoints(accessiblePoints,segments_,position, orientation);
    candidate_points_ = accessiblePoints;

    if(candidate_points_.size() == 0)
    {
        std::cout << "No accessible points" << std::endl;
        return;
    }

    std::cout << "Number of candidates: " << candidate_points_.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const cv::Point2f& point : candidate_points_)
    {
        cv::Point2f current_point = point;

        // // Transform points to experiment coordinates
        // current_point.x = current_point.x * ((double)experiment_width_/image_width_) - 0.5*experiment_width_;
        // current_point.y = current_point.y * ((double)experiment_height_/image_height_) - 0.5*experiment_height_;

        pcl::PointXYZ pcl_point;
        pcl_point.x = static_cast<float>(current_point.x);
        pcl_point.y = static_cast<float>(current_point.y);
        pcl_point.z = 0.0;  // Assuming 2D points with z=0

        targetCloud->points.push_back(pcl_point);
    }

    targetCloud->width = targetCloud->points.size();
    targetCloud->height = 1;

    thinningPointCloud(targetCloud, targetCloud, 0.1);

    _targetCloud = *targetCloud;
    std::cout << "Target cloud generated of size " << _targetCloud.width << std::endl;

    // Save target cloud to file
    // pcl::PCDWriter w;
    // std::string file_name = "/home/michbaum/ElisaSemesterProject/data/pcdData/targetCloud.pcd";
    // w.write<pcl::PointXYZ> (file_name, _targetCloud, false);
}

void TargetGenerator::computeContours()
{
    cv::resize(image_, image_, cv::Size(image_width_, image_height_));

    // Convert the image to grayscale
    cv::Mat imgray;
    cv::cvtColor(image_, imgray, cv::COLOR_BGR2GRAY);

    // Apply thresholding
    cv::Mat thresh;
    cv::threshold(imgray, thresh, 127, 255, 0);

    // See which makes more sense in retrieving segements
    cv::findContours(thresh, contours_, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    //cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
}

// Divide contours into segments < l_max
void TargetGenerator::divideContours()
{
    //std::vector<cv::Point> candidatePoints;
    cv::Point2f current_point;
    cv::Point2f next_point;

    int num_contours = contours_.size();
    for(int i = 0; i < num_contours; i++)
    {
        int j = 0;
        for(j = 0; j < contours_[i].size() - 1; j++)
        {
            // Transform points to experiment coordinates
            // current_point.x = contours_[i][j].x * ((float)experiment_width_/image_width_) - 0.5f*experiment_width_;
            // current_point.y = contours_[i][j].y * ((float)experiment_height_/image_height_) - 0.5f*experiment_height_;

            // next_point.x = contours_[i][j+1].x * ((float)experiment_width_/image_width_) - 0.5f*experiment_width_;
            // next_point.y = contours_[i][j+1].y * ((float)experiment_height_/image_height_) - 0.5f*experiment_height_;

            current_point = contours_[i][j];
            next_point = contours_[i][j+1];

            // Check if distance between points is greater than l_max
            // If so, add new point to contours[i] at index j
            double dist = getDistance(current_point, next_point);
            if(dist > l_max_)
            {
                // Divide contour into segments of length l_max
                // Calculate the number of segments needed
                int num_segments = std::ceil(dist / l_max_);

                // Calculate the segment length
                int segment_length = std::ceil(dist / num_segments);

                cv::Point2f startPoint = current_point;
                cv::Point2f diff = next_point - current_point;

                segments_.push_back(startPoint);
                // Divide contour into segments of length l_max
                for(int k = 1; k < num_segments - 1; k++)
                {
                    // Calculate the position of the new point
                    cv::Point2d new_point = startPoint + diff * (segment_length * k / dist);

                    // Insert the new point at the appropriate index in the contour
                    segments_.push_back(new_point);
                }
                segments_.push_back(next_point);
            }else
            {
                // Add point to candidatePoints
                segments_.push_back(current_point);
            } 
        }
        //current_point.x = contours_[i][j].x * ((float)experiment_width_/image_width_) - 0.5f*experiment_width_;
        //current_point.y = contours_[i][j].y * ((float)experiment_height_/image_height_) - 0.5f*experiment_height_;
        current_point = contours_[i][j];
        segments_.push_back(current_point);
    }

    //return candidatePoints;
}

void TargetGenerator::filterAccesiblePoints(std::vector<cv::Point2f>& accessiblePoints,const std::vector<cv::Point2f>& candidates, Eigen::Vector3d position, Eigen::Quaterniond orientation)
{
    
    //std::vector<cv::Point2f> accessiblePoints;
    // Filter out points that are not accessible

    // Generate a matrix of cv::Points of floorplan that lies within the FoV of the camera

    std:: cout << "Number of FoV candidates: " << candidates.size() << std::endl;

    // Filter out points that are not in FoV
    // Camera Model: getVisibleVoxels()
    std::vector<cv::Point2f> foVPoints;
    std::vector<double> boundaries;
    raycaster_.getVisibleVoxels(&foVPoints,position, orientation, candidates, boundaries);

    double x_min = boundaries[0];
    double x_max = boundaries[1];
    double y_min = boundaries[2];
    double y_max = boundaries[3];

    for(int i = 0; i < foVPoints.size(); i++)
    {
        std::cout << "FoV point: " << foVPoints[i] << std::endl;
    }

    std::cout << "x_min: " << x_min << ", x_max: " << x_max << ", y_min: " << y_min << ", y_max: " << y_max << std::endl;

    // std:: cout << "Number of candidates: " << candidates.size() << std::endl;
    // std::vector<cv::Point> foVPoints(&candidates[0], &candidates[0] + candidates.size() / 10);
    std:: cout << "Number of FoV candidates: " << foVPoints.size() << std::endl;
    if (foVPoints.size() == 0)
    {
        std::cout << "No points in FoV" << std::endl;
        return;
    }

    // Filter out points that cannot be connected by A*
    for(int i = 0; i < foVPoints.size() - 1; i++)
    {
        std::cout << "A* star iteration:" << i << ", with points: " << foVPoints[i] << "&" << foVPoints[i + 1] << std::endl;


        std::vector<cv::Point2f> path;
        bool success = aStar(foVPoints[i], foVPoints[i + 1], path,segments_,x_min, x_max, y_min, y_max, resolution_);
        std::cout << "Success: " << success << std::endl;
        if(success)
        {
            for(int j = 0; j < path.size(); j++)
            {
                accessiblePoints.push_back(path[j]);
            }
        }
        else
        {
            std::cout << "No path found" << std::endl;
        }
    
    }
    
    //accessiblePoints = foVPoints;

}

void TargetGenerator::cleanTargetCloud()
{
    _targetCloud.clear();

    // Clean all temp vectors
    candidate_points_.clear();
}

double TargetGenerator::getDistance(cv::Point2f p1, cv::Point2f p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

std::vector<cv::Point2f> TargetGenerator::getSegments()
{
    return candidate_points_;
}
#ifndef SIMPLE_RAY_CASTER_H
#define SIMPLE_RAY_CASTER_H

#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <opencv2/core/types.hpp>
#include <ros/ros.h>
#include <fstream>

class SimpleRayCaster{
 public:
  SimpleRayCaster(double ray_length,double voxel_size);  // NOLINT
  SimpleRayCaster() = default;

  ~SimpleRayCaster() = default;

  //SimpleRayCaster(const SimpleRayCaster& other);

  SimpleRayCaster& operator=(const SimpleRayCaster& other);

  void initSimpleRayCaster(ros::NodeHandle& nh);

  bool getVisibleVoxels(std::vector<cv::Point2f>* result,
              const Eigen::Vector3d& position,
              const Eigen::Quaterniond& orientation,
              const std::vector<cv::Point2f>& candidates,
              std::vector<double>& boundaries);

  void getDirectionVector(Eigen::Vector3d* result, double relative_x,
                          double relative_y);

  bool containingPoint(const std::vector<cv::Point2f>& points, const Eigen::Vector3d& point, cv::Point2f& result);

  std::ofstream pointsFile_;

 protected:

  // params
  double p_ray_step_;
  double p_downsampling_factor_;  // Artificially reduce the minimum resolution
  // to increase performance

  // constants
  int c_res_x_;  // factual resolution that is used for ray casting
  int c_res_y_;
  
  double p_ray_length_;
  double p_focal_length_;
  int p_resolution_x_;
  int p_resolution_y_;

  double epsilon_ = 0.0001;
    
};

#endif // SIMPLE_RAY_CASTER_H
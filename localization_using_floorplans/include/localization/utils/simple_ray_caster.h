#ifndef SIMPLE_RAY_CASTER_H
#define SIMPLE_RAY_CASTER_H

#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <opencv2/core/types.hpp>
#include <ros/ros.h>

class SimpleRayCaster{
 public:
  SimpleRayCaster(double ray_length,double voxel_size);  // NOLINT
  SimpleRayCaster() = default;

  ~SimpleRayCaster() = default;

  void initSimpleRayCaster(ros::NodeHandle& nh);

  bool getVisibleVoxels(std::vector<cv::Point>* result,
              const Eigen::Vector3d& position,
              const Eigen::Quaterniond& orientation,
              const std::vector<cv::Point>& candidates);

  void getDirectionVector(Eigen::Vector3d* result, double relative_x,
                          double relative_y);

  bool containingPoint(const std::vector<cv::Point>& points, const Eigen::Vector3d& point, cv::Point& result);

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
    
};

#endif // SIMPLE_RAY_CASTER_H
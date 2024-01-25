#ifndef SIMPLE_RAY_CASTER_H
#define SIMPLE_RAY_CASTER_H

#include <vector>
#include <Eigen/Geometry>

class SimpleRayCaster{
 public:
  SimpleRayCaster(double ray_length,double voxel_size);  // NOLINT

  ~SimpleRayCaster() = default;

  bool getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                        const Eigen::Vector3d& position,
                        const Eigen::Quaterniond& orientation);
  void getDirectionVector(Eigen::Vector3d* result, double relative_x,
                          double relative_y);

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
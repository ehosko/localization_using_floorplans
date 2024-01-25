#include "../../../include/localization/utils/simple_ray_caster.h"

SimpleRayCaster::SimpleRayCaster(double ray_length, double voxel_size)
{
    // Constructor
}


void SimpleRayCaster::initSimpleRayCaster(ros::NodeHandle& nh)
{
    // Initialize SimpleRayCaster
    std::cout << "Initializing SimpleRayCaster..." << std::endl;
    nh.getParam("floorplan_node/ray_length", p_ray_length_);
    nh.getParam("floorplan_node/focal_length", p_focal_length_);
    nh.getParam("floorplan_node/resolution_x", p_resolution_x_);
    nh.getParam("floorplan_node/resolution_y", p_resolution_y_);
    nh.getParam("floorplan_node/ray_step", p_ray_step_);
    nh.getParam("floorplan_node/downsampling_factor", p_downsampling_factor_);
    
    double voxel_size = 0.01;
    nh.getParam("floorplan_node/voxel_size", voxel_size);

    // cache param dependent constants
    double c_field_of_view_x = 2.0 * atan2(p_resolution_x_, p_focal_length_ * 2.0);
    double c_field_of_view_y = 2.0 * atan2(p_resolution_y_, p_focal_length_ * 2.0);

    c_res_x_ = std::min(
      static_cast<int>(ceil(p_ray_length_ * c_field_of_view_x /
                            (voxel_size * p_downsampling_factor_))),
      p_resolution_x_);
    c_res_y_ = std::min(
      static_cast<int>(ceil(p_ray_length_ * c_field_of_view_y /
                            (voxel_size * p_downsampling_factor_))),
      p_resolution_y_);
}

bool SimpleRayCaster::getVisibleVoxels(std::vector<cv::Point>* result,
                                       const Eigen::Vector3d& position,
                                       const Eigen::Quaterniond& orientation,
                                       const std::vector<cv::Point>& candidates)
{
    // Naive ray-casting
    Eigen::Vector3d camera_direction;
    Eigen::Vector3d direction;
    Eigen::Vector3d current_position;
    Eigen::Vector3d voxel_center;

    cv::Point observed_point;

    for (int i = 0; i < c_res_x_; ++i) {
        for (int j = 0; j < c_res_y_; ++j) {
        getDirectionVector(
            &camera_direction,
            static_cast<double>(i) / (static_cast<double>(c_res_x_) - 1.0),
            static_cast<double>(j) / (static_cast<double>(c_res_y_) - 1.0));
        direction = orientation * camera_direction;
        double distance = 0.0;
        while (distance < p_ray_length_) {
            current_position = position + distance * direction;
            distance += p_ray_step_;

            // Check voxel occupied
            if (containingPoint(candidates, Eigen::Vector3d(current_position.x(), current_position.y(),0), observed_point)){
                result->push_back(observed_point);
                break;
            }

            // map_->getVoxelCenter(&voxel_center, current_position);
            // result->push_back(voxel_center);
        }
        }
    }
    return true;
}

void SimpleRayCaster::getDirectionVector(Eigen::Vector3d* result, double relative_x,
                                     double relative_y) {
  *result =
      Eigen::Vector3d(p_focal_length_,
                      (0.5 - relative_x) * static_cast<double>(p_resolution_x_),
                      (0.5 - relative_y) * static_cast<double>(p_resolution_y_))
          .normalized();
}

bool SimpleRayCaster::containingPoint(const std::vector<cv::Point>& points, const Eigen::Vector3d& point, cv::Point& result)
{
    for(int i = 0; i < points.size(); i++)
    {
        if(points[i].x == point.x() && points[i].y == point.y())
        {
            result = points[i];
            return true;
        }
    }
    return false;
}
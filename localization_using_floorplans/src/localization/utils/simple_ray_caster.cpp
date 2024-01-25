#include "../../../include/localization/utils/simple_ray_caster.h"

SimpleRayCaster::SimpleRayCaster(double ray_length, double voxel_size)
{
    // Constructor
}

bool SimpleRayCaster::getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                                       const Eigen::Vector3d& position,
                                       const Eigen::Quaterniond& orientation)
{
    // Naive ray-casting
    Eigen::Vector3d camera_direction;
    Eigen::Vector3d direction;
    Eigen::Vector3d current_position;
    Eigen::Vector3d voxel_center;
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

            // // Check voxel occupied
            // if (map_->getVoxelState(current_position) ==
            //     map::OccupancyMap::OCCUPIED) {
            // break;
            // }

            // Add point (duplicates are handled in
            // CameraModel::getVisibleVoxelsFromTrajectory)
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
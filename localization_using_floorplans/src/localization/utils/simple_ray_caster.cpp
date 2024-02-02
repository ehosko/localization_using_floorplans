#include "../../../include/localization/utils/simple_ray_caster.h"

SimpleRayCaster::SimpleRayCaster(double ray_length, double voxel_size)
{
    // Constructor
}


SimpleRayCaster& SimpleRayCaster::operator=(const SimpleRayCaster& other)
{
    // Copy assignment
    return *this;
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

    nh.getParam("floorplan_node/epsilon", epsilon_);

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

    std::string filename = "/home/michbaum/Projects/optag_EH/data/floorplan/points.txt";
    pointsFile_.open(filename);
    if(!pointsFile_.is_open())
    {
        std::cout << "Error opening file" << std::endl;
    }
    else
    {
        pointsFile_ << "x y z" << std::endl;
    }
}

bool SimpleRayCaster::getVisibleVoxels(std::vector<cv::Point2f>* result,
                                       const Eigen::Vector3d& position,
                                       const Eigen::Quaterniond& orientation,
                                       const std::vector<cv::Point2f>& candidates)
{
    // Naive ray-casting
    Eigen::Vector3d camera_direction;
    Eigen::Vector3d direction;
    Eigen::Vector3d current_position;
    Eigen::Vector3d voxel_center;

    cv::Point2f observed_point;

    # pragma omp parallel
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
                if (std::find(result->begin(), result->end(), observed_point) == result->end() ){
                    pointsFile_ << observed_point.x << " " << observed_point.y << " " << 0 << std::endl;
                    result->push_back(observed_point);
                }
                    
                //result->push_back(observed_point);
                break;
            }

            // map_->getVoxelCenter(&voxel_center, current_position);
            // result->push_back(voxel_center);
        }
        }
    }
    pointsFile_.close();
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

bool SimpleRayCaster::containingPoint(const std::vector<cv::Point2f>& points, const Eigen::Vector3d& point, cv::Point2f& result)
{
    // Check if point is contained in vector of points
    //double epsilon = 0.001;

    for(int i = 0; i < points.size(); i++)
    {
        if(std::abs(points[i].x - point.x()) <= epsilon_ && std::abs(points[i].y - point.y()) <= epsilon_)
        {
            result = points[i];
            return true;
        }
    }
    return false;
}
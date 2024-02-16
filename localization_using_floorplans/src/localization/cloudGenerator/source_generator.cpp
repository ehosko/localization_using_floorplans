#include "../../../include/localization/cloudGenerator/source_generator.h"

SourceGenerator::SourceGenerator()
{
    // Constructor
}

SourceGenerator::~SourceGenerator()
{
    // Destructor
}

void SourceGenerator::initSourceGenerator(ros::NodeHandle& nh)
{
    std::cout << "Initializing source generator..." << std::endl;
    XmlRpc::XmlRpcValue T_B_C_xml;
    // Initialize source generator
    if (nh.getParam("floorplan_node/T_B_C", T_B_C_xml)) {

        std::cout << "T_B_C param found" << std::endl;
    
        Eigen::Matrix3d temp_rot_matrix = Eigen::Matrix3d::Identity();
        Eigen::Vector3d temp_translation = Eigen::Vector3d::Zero();
        
        if (T_B_C_xml.size() != 4) {
            std::cout << "T_B_C has " << T_B_C_xml.size() << " elements" << std::endl;
            return;
        }
        // read raw inputs
        for (size_t i = 0; i < 3; ++i) {
            if (T_B_C_xml[i].size() != 4) {
             std::cout << "XmlRpc matrix has " << T_B_C_xml[i].size()
                        << " columns in its " << i << " row";
            return;
            }
            for (size_t j = 0; j < 3; ++j) {
                temp_rot_matrix(i, j) = static_cast<double>(T_B_C_xml[i][j]);
            }
                temp_translation(i) = static_cast<double>(T_B_C_xml[i][3]);
        }

        // Construct and normalize
        Eigen::Quaterniond temp_quaternion(temp_rot_matrix);
        temp_quaternion.normalize();
        T_B_C_ = Eigen::Translation3d(temp_translation) * temp_quaternion;
        
        std::cout << "T_B_C: " << T_B_C_(0,0) << " " << T_B_C_(0,1) << " " << T_B_C_(0,2) << std::endl;

        // See if we need to invert it.
        bool invert_static_tranform = false;
        nh.param("floorplan_node/invert_T_B_C", invert_static_tranform,
                            invert_static_tranform);
        if (invert_static_tranform) {
            T_B_C_ = T_B_C_.inverse();
        }
    }

    transformSub_ = nh.subscribe("/rovioli/Transform_G_I", 1, &SourceGenerator::transformCallback, this);
}

void SourceGenerator::transformCallback(const geometry_msgs::TransformStamped& transform_msg)
{
    // Transform callback
    // Eigen::Vector3d position(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z);
    // Eigen::Quaterniond orientation(transform_msg.transform.rotation.w, transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z);

    // transform_ = Eigen::Translation3d(position) * orientation;

    // std::cout << "Transform received: " << transform_(0,0) << " " << transform_(0,1) << " " << transform_(0,2) << std::endl;

    transform_msg_.push_back(transform_msg);
}

void SourceGenerator::generateSourceCloud(pcl::PointCloud<pcl::PointXYZ> depthCloud, Eigen::Quaterniond q, const ros::Time& timestamp)
{
    // Generate source cloud
    std::cout << "Generating source cloud..." << std::endl;

    // bool match_found = false;
    // if(depthCloud_msg_.size() == 0)
    // {
    //     ROS_ERROR("No depth cloud messages received.");
    //     return;
    // }
    // std::vector<sensor_msgs::PointCloud2ConstPtr>::iterator it =
    //     depthCloud_msg_.begin();
    // for (; it != depthCloud_msg_.end(); ++it) {
    //     const sensor_msgs::PointCloud2& pointCloud = **it;
    //     if (pointCloud.header.stamp > timestamp) {
    //         if ((pointCloud.header.stamp - timestamp).toNSec() < timestamp_tolerance_ns_) {
    //             match_found = true;
    //         }
    //         break;
    //     }

    //     if ((timestamp - pointCloud.header.stamp).toNSec() < timestamp_tolerance_ns_) {
    //         match_found = true;
    //         break;
    //     }
    // }
    
    // if(match_found){
    //     std::cout<< "Matching point cloud found" << std::endl;
    //     pcl::fromROSMsg(**it, depthCloud);
    // }else
    // {
    //     try{
    //         const sensor_msgs::PointCloud2& pointCloud = **(it-1);
    //         std::cout << pointCloud.header.stamp << " vs " << timestamp << std::endl;
    //     //ROS_ERROR("No matching point cloud found.");
    //     }
    //     catch(const std::exception& e)
    //     {
    //         std::cerr << e.what() << '\n';
    //     }
        
    // }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloudTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    TransformPoints(depthCloud, depthCloudTransformed, q, timestamp);
    ProjectOnFloor(*depthCloudTransformed, sourceCloud, q);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudThinned(new pcl::PointCloud<pcl::PointXYZ>);

    thinningPointCloud(sourceCloud, sourceCloudThinned, 0.1);

    _sourceCloud = *sourceCloudThinned;

    std::cout << "Source cloud generated of size " << _sourceCloud.width << std::endl;
}


std::vector<double> MinMaxPt(pcl::PointCloud<pcl::PointXYZ> depthCloud)
{
    // Compute min and max points
    std::vector<double> minMaxPt;
    double minX = 1000.0;
    double minY = 1000.0;
    double maxX = -1000.0;
    double maxY = -1000.0;
    for(int i = 0; i < depthCloud.width; i++)
    {
        if(depthCloud.points[i].x < minX)
        {
            minX = depthCloud.points[i].x;
        }
        if(depthCloud.points[i].x > maxX)
        {
            maxX = depthCloud.points[i].x;
        }
        if(depthCloud.points[i].y < minY)
        {
            minY = depthCloud.points[i].y;
        }
        if(depthCloud.points[i].y > maxY)
        {
            maxY = depthCloud.points[i].y;
        }
    }
    minMaxPt.push_back(minX);
    minMaxPt.push_back(minY);
    minMaxPt.push_back(maxX);
    minMaxPt.push_back(maxY);

    return minMaxPt;
}

void SourceGenerator::TransformPoints(pcl::PointCloud<pcl::PointXYZ> depthCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloudTransform,Eigen::Quaterniond q, const ros::Time& timestamp)
{
    bool match_found = false;
    std::vector<geometry_msgs::TransformStamped>::iterator it =
        transform_msg_.begin();
    for (; it != transform_msg_.end(); ++it) {
        // If the current transform is newer than the requested timestamp, we need
        // to break.
        if (it->header.stamp > timestamp) {
            if ((it->header.stamp - timestamp).toNSec() < timestamp_tolerance_ns_) {
                match_found = true;
            }
            break;
        }

        if ((timestamp - it->header.stamp).toNSec() < timestamp_tolerance_ns_) {
            match_found = true;
            break;
        }
    }

    if (match_found) {
        
        geometry_msgs::TransformStamped transform_msg = *it;
        Eigen::Vector3d position(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z);
        Eigen::Quaterniond orientation(transform_msg.transform.rotation.w, transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z);
        transform_ = Eigen::Translation3d(position) * orientation;
  
    } else {
        // If we think we have an inexact match, have to check that we're still
        // within bounds and interpolate.
        if (it == transform_msg_.begin() || it == transform_msg_.end()) {
            ROS_ERROR("No matching transform found.");
         //return false;
        }
        // Newest should be 1 past the requested timestamp, oldest should be one
        // before the requested timestamp.
        Eigen::Transform<double, 3, Eigen::Affine> transform_newest;
        geometry_msgs::TransformStamped transform_msg = *it;
        Eigen::Vector3d position(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z);
        Eigen::Quaterniond orientation(transform_msg.transform.rotation.w, transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z);
        transform_newest = Eigen::Translation3d(position) * orientation;

        // int64_t offset_newest_ns = (it->header.stamp - timestamp).toNSec();
        // // We already checked that this is not the beginning.
        // it--;
        // Eigen::Affine3d transform_oldest;
        // transform_msg = *it;
        // position = Eigen::Vector3d(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z);
        // orientation = Eigen::Quaterniond(transform_msg.transform.rotation.w, transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z);

        // transform_oldest = Eigen::Translation3d(position) * orientation;

        // int64_t offset_oldest_ns = (timestamp - it->header.stamp).toNSec();

        // // Interpolate between the two transformations using the exponential map.
        // float t_diff_ratio =
        //     static_cast<float>(offset_oldest_ns) /
        //     static_cast<float>(offset_newest_ns + offset_oldest_ns);

        // Eigen::Vector6 diff_vector =
        //     (T_G_D_oldest.inverse() * T_G_D_newest).log();
        // transform_ = transform_oldest * Transformation::exp(t_diff_ratio * diff_vector);

        transform_ = transform_newest;
    }

    // Transform points
    for(int i = 0; i < depthCloud.width; i++)
    {
        Eigen::Vector3d point(depthCloud.points[i].x, depthCloud.points[i].y, depthCloud.points[i].z);
        Eigen::Vector3d pointTransformed = transform_ * T_B_C_ * point;
        depthCloudTransform->push_back(pcl::PointXYZ(pointTransformed(0), pointTransformed(1), pointTransformed(2)));
    }
}

void SourceGenerator::ProjectOnFloor(pcl::PointCloud<pcl::PointXYZ> depthCloudTransformed,pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, Eigen::Quaterniond q)
{
    // Transform depth cloud
    // pcl::PointCloud<pcl::PointXYZ> depthCloudTransformed;
    // for(int i = 0; i < depthCloud.width; i++)
    // {
    //     Eigen::Vector3d point(depthCloud.points[i].x, depthCloud.points[i].y, depthCloud.points[i].z);
    //     Eigen::Vector3d pointTransformed = transform_ * T_B_C_ * point;
    //     depthCloudTransformed.push_back(pcl::PointXYZ(pointTransformed(0), pointTransformed(1), pointTransformed(2)));
    // }

    // for(int i = 0; i < 5; i++)
    // {
    //     std::cout << "Depth cloud point " << i << " x: " << depthCloudTransformed.points[i].x << "  y: " << depthCloudTransformed.points[i].y << "  z: " << depthCloudTransformed.points[i].z<< std::endl;

    //     std::cout << "Depth cloud point " << depthCloudTransformed.width - i -1 << " x: " << depthCloudTransformed.points[depthCloudTransformed.width - i -1].x << "  y: " << depthCloudTransformed.points[depthCloudTransformed.width - i -1].y << "  z: " << depthCloudTransformed.points[depthCloudTransformed.width - i -1].z<< std::endl;
    // }

    std::vector<double> minMaxPt = MinMaxPt(depthCloudTransformed);
    ///std::cout << "Min x: " << minMaxPt[0] << "  Min y: " << minMaxPt[1] << "  Max x: " << minMaxPt[2] << "  Max y: " << minMaxPt[3] << std::endl;
    
    // Compute normal vector of floor
    Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
    Eigen::Vector3d normalVector(0.0, 0.0, 1.0); // Assuming floor is horizontal
    normalVector = rotationMatrix * normalVector;

    // Project depth cloud on floor
    pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *depthCloudPtr = depthCloudTransformed;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudProjectedPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.resize(4);
    coefficients->values[0] = normalVector(0);
    coefficients->values[1] = normalVector(1);
    coefficients->values[2] = normalVector(2);
    coefficients->values[3] = 0.0;
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(depthCloudPtr);
    proj.setModelCoefficients(coefficients);
    proj.filter(*sourceCloud);
    //_sourceCloud = *sourceCloudProjectedPtr;

    // Set z to 0
    for(int i = 0; i < sourceCloud->width; i++)
    {
        sourceCloud->points[i].z = 0.0;
    }

    std::cout << sourceCloud->points[0].z << std::endl;

    // for(int i = 0; i < 5; i++)
    // {
    //     std::cout << "Projected depth cloud point " << i << " x: " << sourceCloud->points[i].x << "  y: " << sourceCloud->points[i].y << "  z: " << sourceCloud->points[i].z<< std::endl;
    //     int width = sourceCloud->width;
    //     std::cout << "Projected depth cloud point " << width - i -1 << " x: " << sourceCloud->points[width - i -1].x << "  y: " << sourceCloud->points[width - i -1].y << "  z: " << sourceCloud->points[width - i -1].z<< std::endl;
    // }
    // std::vector<double> minMaxPtS = MinMaxPt(*sourceCloud);
    // std::cout << "Min x: " << minMaxPtS[0] << "  Min y: " << minMaxPtS[1] << "  Max x: " << minMaxPtS[2] << "  Max y: " << minMaxPtS[3] << std::endl;

}

void SourceGenerator::projectPosOnFloor(Eigen::Vector3d position, Eigen::Quaterniond q)
{
    std::cout << "Projecting position on floor..." << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

    // Project position on floor
    pcl::PointCloud<pcl::PointXYZ> positionCloud;
    positionCloud.push_back(pcl::PointXYZ(position(0), position(1), position(2)));
    *depthCloudPtr = positionCloud;

    // Compute normal vector of floor
    Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
    Eigen::Vector3d normalVector(0.0, 0.0, 1.0); // Assuming floor is horizontal
    normalVector = rotationMatrix * normalVector;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.resize(4);
    coefficients->values[0] = normalVector(0);
    coefficients->values[1] = normalVector(1);
    coefficients->values[2] = normalVector(2);
    coefficients->values[3] = 0.0;
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(depthCloudPtr);
    proj.setModelCoefficients(coefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
    proj.filter(*sourceCloud);

    std::cout<< "Cloud Width " << sourceCloud->width << std::endl;

    //projected_pos = Eigen::Translation3d(sourceCloud->points[0].x, sourceCloud->points[0].y, sourceCloud->points[0].z) * q.normalized();

}


void SourceGenerator::cleanSourceCloud()
{
    // Clean source cloud
    _sourceCloud.clear();
}
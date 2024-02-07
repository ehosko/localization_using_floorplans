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
    Eigen::Vector3d position(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z);
    Eigen::Quaterniond orientation(transform_msg.transform.rotation.w, transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z);

    transform_ = Eigen::Translation3d(position) * orientation;

    std::cout << "Transform received: " << transform_(0,0) << " " << transform_(0,1) << " " << transform_(0,2) << std::endl;
}

void SourceGenerator::generateSourceCloud(pcl::PointCloud<pcl::PointXYZ> depthCloud, Eigen::Quaterniond q)
{
    // Generate source cloud
    std::cout << "Generating source cloud..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
    ProjectOnFloor(depthCloud, sourceCloud, q);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudThinned(new pcl::PointCloud<pcl::PointXYZ>);

    thinningPointCloud(sourceCloud, sourceCloudThinned, 0.1);

    _sourceCloud = *sourceCloudThinned;

    std::cout << "Source cloud generated of size " << _sourceCloud.width << std::endl;
}

void SourceGenerator::ProjectOnFloor(pcl::PointCloud<pcl::PointXYZ> depthCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, Eigen::Quaterniond q)
{
    // Transform depth cloud
    pcl::PointCloud<pcl::PointXYZ> depthCloudTransformed;
    for(int i = 0; i < depthCloud.width; i++)
    {
        Eigen::Vector3d point(depthCloud.points[i].x, depthCloud.points[i].y, depthCloud.points[i].z);
        Eigen::Vector3d pointTransformed = transform_ * T_B_C_ * point;
        depthCloudTransformed.push_back(pcl::PointXYZ(pointTransformed(0), pointTransformed(1), pointTransformed(2)));
    }

    for(int i = 0; i < 5; i++)
    {
        std::cout << "Depth cloud point " << i << " x: " << depthCloudTransformed.points[i].x << "  y: " << depthCloudTransformed.points[i].y << "  z: " << depthCloudTransformed.points[i].z<< std::endl;

        std::cout << "Depth cloud point " << depthCloudTransformed.width - i -1 << " x: " << depthCloudTransformed.points[depthCloudTransformed.width - i -1].x << "  y: " << depthCloudTransformed.points[depthCloudTransformed.width - i -1].y << "  z: " << depthCloudTransformed.points[depthCloudTransformed.width - i -1].z<< std::endl;
    }
    
    // Compute normal vector of floor
    Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
    Eigen::Vector3d normalVector(0.0, 0.0, 1.0); // Assuming floor is horizontal
    normalVector = rotationMatrix * normalVector;

    // Project depth cloud on floor
    pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *depthCloudPtr = depthCloud;
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
    for(int i = 0; i < _sourceCloud.width; i++)
    {
        sourceCloud->points[i].z = 0.0;
    }

    std::cout << "Depth cloud projected on floor of size " << _sourceCloud.width << std::endl;
    //*sourceCloud = *sourceCloudProjectedPtr; 
}

void SourceGenerator::RemoveOutliers()
{
    // Remove outliers
    // std::cout << "Removing outliers..." << std::endl;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    // *sourceCloudPtr = _sourceCloud;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudFilteredPtr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(sourceCloudPtr);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(1.0);
    // sor.filter(*sourceCloudFilteredPtr);
    // _sourceCloud = *sourceCloudFilteredPtr;
    // std::cout << "Outliers removed from source cloud of size " << _sourceCloud.width << std::endl;
}

void SourceGenerator::RemoveOutliersPlaneInformation()
{
    // Remove outliers using plane information
    // std::cout << "Removing outliers using plane information..." << std::endl;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    // *sourceCloudPtr = _sourceCloud;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudFilteredPtr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // coefficients->values.resize(4);
    // coefficients->values[0] = 0.0;
    // coefficients->values[1] = 0.0;
    // coefficients->values[2] = 1.0;
    // coefficients->values[3] = 0.0;
    // pcl::ProjectInliers<pcl::PointXYZ> proj;
    // proj.setModelType(pcl::SACMODEL_PLANE);
    // proj.setInputCloud(sourceCloudPtr);
    // proj.setModelCoefficients(coefficients);
    // proj.filter(*sourceCloudFilteredPtr);
    // _sourceCloud = *sourceCloudFilteredPtr;
    // std::cout << "Outliers removed from source cloud of size " << _sourceCloud.width << std::endl;
}

void SourceGenerator::RemoveOutliersGICP()
{
    // Remove outliers using GICP
    // std::cout << "Removing outliers using GICP..." << std::endl;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    // *sourceCloudPtr = _sourceCloud;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudFilteredPtr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloudFilteredPtr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(sourceCloudPtr);
    // icp.setInputTarget(targetCloudPtr);
    // icp.align(*sourceCloudFilteredPtr);
    // _sourceCloud = *sourceCloudFilteredPtr;
    // std::cout << "Outliers removed from source cloud of size " << _sourceCloud.width << std::endl;
}   


void SourceGenerator::cleanSourceCloud()
{
    // Clean source cloud
    _sourceCloud.clear();
}
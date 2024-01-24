#include "../../../include/localization/cloudGenerator/source_generator.h"

SourceGenerator::SourceGenerator()
{
    // Constructor
}

SourceGenerator::~SourceGenerator()
{
    // Destructor
}

void SourceGenerator::ProjectOnFloor(pcl::PointCloud<pcl::PointXYZ> depthCloud,Eigen::Quaterniond q)
{
    // Project depth cloud on floor
    std::cout << "Projecting depth cloud on floor..." << std::endl;
    
    // Compute normal vector of floor
    Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
    Eigen::Vector3d normalVector(0.0, 0.0, 1.0); // Assuming floor is horizontal
    normalVector = rotationMatrix * normalVector;

    // Project depth cloud on floor
    pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *depthCloudPtr = depthCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudProjectedPtr(new pcl::PointCloud<pcl::PointXYZ>);
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
    proj.filter(*sourceCloudProjectedPtr);
    _sourceCloud = *sourceCloudProjectedPtr;

    std::cout << "Depth cloud projected on floor of size " << _sourceCloud.width << std::endl;
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
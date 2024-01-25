#include "../../include/localization/simple_localizer.h"
// #include "../../include/localization/cloudGenerator/source_generator.h"
// #include "../../include/localization/cloudGenerator/target_generator.h"


SimpleLocalizer::SimpleLocalizer()
{
    // Constructor
}

SimpleLocalizer::~SimpleLocalizer()
{
    // Destructor
}

void SimpleLocalizer::setupFromParam()
{
    // TODO implement
    std::string floorplan_path;
    double ray_length;

    std::cout << "Setting up from param..." << std::endl;

    nh_.getParam("floorplan_node/floorplan_path", floorplan_path);
    nh_.getParam("floorplan_node/ray_length", ray_length);

    sourceGenerator_ = SourceGenerator();
    targetGenerator_ = TargetGenerator(floorplan_path);


    std::cout << "Floorplan path: " << floorplan_path << std::endl;
    std::cout << "Ray length: " << ray_length << std::endl;
}

int SimpleLocalizer::computeTransformationGICP(pcl::PointCloud<pcl::PointXYZ> sourceCloud, 
                                                pcl::PointCloud<pcl::PointXYZ> targetCloud,
                                                Eigen::Matrix4f& transformationMatrix)
{
    // Compute transformation matrix using GICP
    std::cout << "Computing transformation matrix using GICP..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *sourceCloudPtr = sourceCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *targetCloudPtr = targetCloud;

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(sourceCloudPtr);
    gicp.setInputTarget(targetCloudPtr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_source =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    gicp.align(*aligned_source);

    transformationMatrix = gicp.getFinalTransformation();
  

    return gicp.hasConverged();
}

void SimpleLocalizer::localize()
{
    // Set up publisher and subscriber

    // while(ros::ok)
    // {
    //     std::cout << "Localizing..." << std::endl;
    // }
}




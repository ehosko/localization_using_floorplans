#include "../../../include/localization/utils/process_point_cloud.h"

void thinningPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr thinnedCloud, double leaf_size){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> grid;

    grid.setInputCloud (cloud);
    grid.setLeafSize (leaf_size, leaf_size, 0);

    std::cout << "Input cloud size: " << cloud->width << std::endl;
    grid.filter (*cloud_filtered);
    std::cout << "Filtered cloud size: " << cloud_filtered->width << std::endl;

    *thinnedCloud = *cloud_filtered;
}
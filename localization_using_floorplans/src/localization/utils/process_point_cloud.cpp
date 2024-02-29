#include "../../../include/localization/utils/process_point_cloud.h"

void thinningPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr thinnedCloud, double leaf_size)
{
    // Thinning point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> grid;

    grid.setInputCloud (cloud);
    grid.setLeafSize (leaf_size, leaf_size, 0);

    grid.filter (*cloud_filtered);

    *thinnedCloud = *cloud_filtered;
}
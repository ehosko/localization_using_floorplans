#ifndef PROCESS_POINT_CLOUD_H
#define PROCESS_POINT_CLOUD_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>

void thinningPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr thinnedCloud, double leaf_size);

#endif // PROCESS_POINT_CLOUD_H
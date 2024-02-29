#ifndef ASTAR_H
#define ASTAR_H

#include <opencv2/core/types.hpp>
#include <stdlib.h>

#include <queue>
#include <vector>
#include <unordered_map>

#include <eigen3/Eigen/Geometry>

bool aStar(cv::Point2f, cv::Point2f, std::vector<cv::Point2f>&,std::vector<cv::Point2f> segments,Eigen::Vector3d pos,double x_min, double x_max, double y_min, double y_max, double resolution, double epsilon);

double getDistance(cv::Point2f, cv::Point2f);

#endif // ASTAR_H
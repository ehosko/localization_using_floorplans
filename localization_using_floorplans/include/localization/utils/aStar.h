#ifndef ASTAR_H
#define ASTAR_H

#include <opencv2/core/types.hpp>
#include <stdlib.h>

#include <queue>
#include <vector>
#include <unordered_map>

bool aStar(cv::Point2f, cv::Point2f, std::vector<cv::Point2f>&,std::vector<cv::Point2f> segments,double x_min, double x_max, double y_min, double y_max, double resolution);

double getDistance(cv::Point2f, cv::Point2f);

#endif // ASTAR_H
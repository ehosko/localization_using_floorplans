#ifndef FLOORPLANGRAPH_H
#define FLOORPLANGRAPH_H

#include <vector>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include "tsp_solver.h"

struct Node
{
    double x;
    double y;
    int i;
    int j;
    //std::vector<Node*> neighbors;

    bool operator==(const Node& other) const
    {
        return (i == other.i && j == other.j);
    }

};

class FloorplanGraph
{
public:
    FloorplanGraph();
    ~FloorplanGraph();

    void initFloorplanGraph(ros::NodeHandle& nh);

    void setWeightedEdgeMatrix(Eigen::Matrix2d weightedAdjacenyMatrix) { weightedEdgeMatrix_ = weightedAdjacenyMatrix; };
    
private:

    void buildGraph();
    double aStar(Node start, Node goal);
    std::pair<double,double> transformGridToMap(int i, int j);
    std::pair<int,int> transformMapToGrid(double x, double y);

    bool neighborsOccupied(int i, int j, int radius);

    int image_width_ = 0; //Image width
    int image_height_ = 0; //Image height

    int experiment_width_ = 0; //Floorplan width
    int experiment_height_ = 0; //Floorplan height

    double resolution_ = 0.1; //Resolution of the floorplan

    double threshold_ = 5;

    std::string lkh_executable_;

    cv::Mat map_;
    Eigen::MatrixXi occupancyGrid_;

    int num_samples_ = 0;
    int skip_distance_ = 0;

    std::vector<Node> nodes_;
    Eigen::MatrixXd weightedEdgeMatrix_;

    std::string edge_log_file_;
    std::string node_log_file_;
    std::string path_log_file_;
    
};

#endif // "FLOORPLANGRAPH_H"
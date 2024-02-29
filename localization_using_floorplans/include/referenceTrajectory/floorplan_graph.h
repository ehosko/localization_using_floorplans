#ifndef FLOORPLANGRAPH_H
#define FLOORPLANGRAPH_H

#include <vector>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <nav_msgs/Odometry.h>

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

    double getDistance(Node n)
    {
        return sqrt(pow(x - n.x, 2) + pow(y - n.y, 2));
    }

};

class FloorplanGraph
{
public:
    FloorplanGraph() : tfListener_(tfBuffer_) {};
    ~FloorplanGraph();

    void initFloorplanGraph(ros::NodeHandle& nh);

    void setWeightedEdgeMatrix(Eigen::Matrix2d weightedAdjacenyMatrix) { weightedEdgeMatrix_ = weightedAdjacenyMatrix; };
        
    std::ofstream path_file_;
    std::ofstream occupancy_file_;
    std::ofstream occupancy_opt_file_;
private:

    void buildGraph();
    std::pair<double,double> transformGridToMap(int i, int j);
    std::pair<int,int> transformMapToGrid(double x, double y);

    bool isEdgeClear(Node start, Node goal);

    bool neighborsOccupied(int i, int j, int radius);

    void odomCallback(const nav_msgs::Odometry& msg);

    bool reachedCurrentNode(Eigen::Vector3d pos, Eigen::Quaterniond q);
    //void getNextNode(Eigen::Vector3d& pos);
    void broadcastTransform(Node nextNode);

    void buildOccupancyGrid(std::vector<std::vector<cv::Point>> contours);
    void sampleNodes_uniform();
    void sampleNodes_random();

    bool rewirePath(Eigen::Vector3d currentPos,double distance);

    void logOccupancy(Eigen::Vector3d projectedVector);

    ros::Publisher ready_pub_;
    ros::Subscriber odom_sub_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster broadcaster_;

    int image_width_ = 0; //Image width
    int image_height_ = 0; //Image height

    int experiment_width_ = 0; //Floorplan width
    int experiment_height_ = 0; //Floorplan height

    double resolution_ = 0.1; //Resolution of the floorplan

    std::string lkh_executable_; // LKH executable

    cv::Mat map_; //OpenCV Image
    Eigen::MatrixXi occupancyGrid_; //Occupancy Grid

    std::vector<int>* tour_ = new std::vector<int>; // Optimal path
    int next_node_counter_ = 0; // Counter for next node to visit

    std::vector<int> nodes_visited_; // Nodes visited
    std::vector<int> nodes_to_visit_;

    double radius_; // Radius to consider node as visited
    double max_radius_; // Max radius to next node before rewirring

    int num_samples_ = 0; // Number of nodes to sample
    int skip_distance_ = 0; // Skip distance  from occupied voxels for traversibility

    int occupancy_radius_ = 0; // Radius to consider node as occupied

    bool sample_uniform_ = false; // Sample nodes uniformly or randomly

    bool contours_traversable_ = false; // If contours are traversable
    double contour_cost_factor_ = 300; // If contours are traversable - multiplication factor of traversability cost

    //normal vector of the floor
    Eigen::Vector3d floorNormal_ = Eigen::Vector3d(0.0, 0.0, 1.0);

    std::vector<Node> nodes_; // Nodes
    Eigen::MatrixXd weightedEdgeMatrix_; // Weighted Edge Matrix

    std::string edge_log_file_;
    std::string node_log_file_;
    std::string opt_path_log_file_;
    std::string path_log_file_;

    std::string occupancy_log_file_; // Collect coverage per time
    std::string occupancy_opt_log_file_;

    // To convert from image to map
    double x_min_ = 0; // Min x of environment 
    double x_max_ = 0; // Max x of environment
    double y_min_ = 0; // Min y of environment
    double y_max_ = 0; // Max y of environment

    double start_x_ = 0; // Start x of agent
    double start_y_ = 0; // Start y of agent
};

#endif // "FLOORPLANGRAPH_H"
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

    double threshold_ = 5;

    std::string lkh_executable_;

    cv::Mat map_;
    Eigen::MatrixXi occupancyGrid_;

    std::vector<int>* tour_ = new std::vector<int>;
    int next_node_counter_ = 0;

    std::vector<int> nodes_visited_;
    std::vector<int> nodes_to_visit_;

    double radius_;
    double max_radius_;
    int num_samples_ = 0;
    int skip_distance_ = 0;

    int occupancy_radius_ = 0;

    bool sample_uniform_ = false;

    bool contours_traversable_ = false;
    double contour_cost_factor_ = 300;

    //normal vector of the floor
    Eigen::Vector3d floorNormal_ = Eigen::Vector3d(0.0, 0.0, 1.0);

    std::vector<Node> nodes_;
    Eigen::MatrixXd weightedEdgeMatrix_;

    std::string edge_log_file_;
    std::string node_log_file_;
    std::string opt_path_log_file_;
    std::string path_log_file_;

    std::string occupancy_log_file_;
    std::string occupancy_opt_log_file_;

    double x_min_ = 0;
    double x_max_ = 0;
    double y_min_ = 0;
    double y_max_ = 0;

    double start_x_ = 0;
    double start_y_ = 0;
};

#endif // "FLOORPLANGRAPH_H"
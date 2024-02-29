#include "../../include/referenceTrajectory/floorplan_graph.h"


// FloorplanGraph::FloorplanGraph()
// {
//     // Constructor
// }

FloorplanGraph::~FloorplanGraph()
{
    // Destructor
}

void FloorplanGraph::initFloorplanGraph(ros::NodeHandle& nh)
{
    std::string floorplan_path;
    nh.getParam("floorplan_node/floorplan_path", floorplan_path);

    map_ = cv::imread(floorplan_path);

    nh.getParam("floorplan_node/floorplan_width", experiment_width_);
    nh.getParam("floorplan_node/floorplan_height", experiment_height_);

    nh.getParam("floorplan_node/voxel_size", resolution_);

    nh.getParam("floorplan_node/num_samples", num_samples_);
    nh.getParam("floorplan_node/occupancy_threshold", occupancy_radius_);

    nh.getParam("floorplan_node/radius", radius_);
    nh.getParam("floorplan_node/max_radius", max_radius_);

    nh.getParam("floorplan_node/skip_distance", skip_distance_);

    nh.getParam("floorplan_node/lkh_executable", lkh_executable_);

    // nh.getParam("floorplan_node/edge_log_file", edge_log_file_);
    // nh.getParam("floorplan_node/node_log_file", node_log_file_);
    nh.getParam("floorplan_node/opt_path_log_file", opt_path_log_file_);
    // nh.getParam("floorplan_node/path_log_file", path_log_file_);

    nh.getParam("floorplan_node/occupancy_log_file", occupancy_log_file_);
    nh.getParam("floorplan_node/occupancy_opt_log_file", occupancy_opt_log_file_);

    nh.getParam("floorplan_node/x_min", x_min_);
    nh.getParam("floorplan_node/x_max", x_max_);
    nh.getParam("floorplan_node/y_max", y_max_);
    nh.getParam("floorplan_node/y_min", y_min_);

    nh.getParam("floorplan_node/start_x", start_x_);
    nh.getParam("floorplan_node/start_y", start_y_);

    nh.getParam("floorplan_node/contours_traversable", contours_traversable_);
    nh.getParam("floorplan_node/contour_cost_factor", contour_cost_factor_);

    ROS_DEBUG("Floorplan width %d", experiment_width_);
    ROS_DEBUG("Floorplan height %d", experiment_height_);


    // Create log files
    occupancy_file_.open(occupancy_log_file_);
    if(!occupancy_file_.is_open())
    {
        std::cout << "Error opening file" << std::endl;
    }
    else
    {
        occupancy_file_ << "time,occupancy" << std::endl;
    }

    occupancy_opt_file_.open(occupancy_opt_log_file_);
    if(!occupancy_opt_file_.is_open())
    {
        std::cout << "Error opening file" << std::endl;
    }
    else
    {
        occupancy_opt_file_ << "time,occupancy" << std::endl;
    }

    std::string sample_method;
    nh.getParam("floorplan_node/sample_method", sample_method);
    if(sample_method == "uniform")
    {
        sample_uniform_ = true;
    }
    else
    {
        sample_uniform_ = false;
    }

    image_height_ = experiment_height_ / resolution_;
    image_width_ = experiment_width_ / resolution_;

    ready_pub_ = nh.advertise<std_msgs::String>("floorplan_node/opt_traj_ready", 10);

    buildGraph();

    odom_sub_ = nh.subscribe("odometry", 1, &FloorplanGraph::odomCallback, this);

}

void FloorplanGraph::odomCallback(const nav_msgs::Odometry& msg)
{
    Eigen::Vector3d position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    Eigen::Quaterniond orientation(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

    // See if current node reached
    if(reachedCurrentNode(position,orientation)){
        // Counter already increased during check
        broadcastTransform(nodes_[tour_->at(next_node_counter_)]);
    }

    // Log occupancy in following optimal path - NOT the same as overall occupancy
    double occ = (double)next_node_counter_/nodes_.size();
    occupancy_opt_file_ << ros::Time::now().toSec() << "," << occ << std::endl;
}

bool FloorplanGraph::rewirePath(Eigen::Vector3d projectedVector, double distance)
{
    // Get nearest node out of not visited nodes
    double min_distance = distance;
    int min_indx = -1;
    for(int i = 0; i < tour_->size(); i++)
    {
        int indx = tour_->at(i);
        Node node = nodes_[indx];
        double distance = (Eigen::Vector3d(node.x, node.y, 0) - projectedVector).norm();
        if(distance < min_distance && std::find(nodes_to_visit_.begin(), nodes_to_visit_.end(),indx)!=nodes_to_visit_.end())
        {
            min_distance = distance;
            min_indx = i;
        }
    }

    if(min_indx != -1)
    {
        next_node_counter_ = min_indx;
        return true;
    }

    return false;
}

bool FloorplanGraph::reachedCurrentNode(Eigen::Vector3d pos, Eigen::Quaterniond q)
{

    //Project position onto floor
    // Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
    // Eigen::Vector3d normalVector(0.0, 0.0, 1.0); // Assuming floor is horizontal
    // normalVector = rotationMatrix * normalVector;

    // // Project the point onto the floor
    // double distance = -normalVector.dot(pos);
    // Eigen::Vector3d projectedVector = pos - distance * normalVector;

    // Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
    // Eigen::Vector3d projectedVector = rotationMatrix.transpose() * pos;

    // orthogonol projection
    Eigen::Vector3d projectedVector = pos;
    projectedVector.z() = 0;

    // Check if reached end
    if(next_node_counter_ >= tour_->size())
    {
        return false;
    }
    
    int indx = tour_->at(next_node_counter_);
    Node currentNode =  nodes_[indx];

    //Log occupancy
    logOccupancy(projectedVector);
    
    //Check if position in radius of current Node
    if(std::abs(projectedVector.x() - currentNode.x) < radius_ && std::abs(projectedVector.y() - currentNode.y) < radius_)
    {
        // Check if following nodes also already near position
        while(std::abs(projectedVector.x() - currentNode.x) < radius_ && std::abs(projectedVector.y() - currentNode.y) < radius_){
            next_node_counter_++;
            if(next_node_counter_ >= tour_->size()){
                return false;
            }
            indx = tour_->at(next_node_counter_);
            currentNode =  nodes_[indx];
        }
        
        // If in nodes to visit continue following path
        if(std::find(nodes_to_visit_.begin(), nodes_to_visit_.end(),indx)!=nodes_to_visit_.end())
        {
            return true;
        }

        // If not in nodes to visit, rewire path
        return rewirePath(projectedVector, std::numeric_limits<double>::max());
    }

    Eigen::Vector3d nextNode = {currentNode.x, currentNode.y, 0};
    //Check if postion too far from next node -> rewire
    double distance_to_next = (nextNode - projectedVector).norm();
    if(distance_to_next > max_radius_)
    {
        return rewirePath(projectedVector, distance_to_next);
    }

    return false;
}

void FloorplanGraph::logOccupancy(Eigen::Vector3d projectedVector)
{
    //Log total occupancy
    for(int i = 0; i < nodes_to_visit_.size(); i++){
        Node node = nodes_[nodes_to_visit_[i]];
        if(std::abs(projectedVector.x() - node.x) < radius_ && std::abs(projectedVector.y() - node.y) < radius_)
            nodes_to_visit_.erase(nodes_to_visit_.begin() + i);   
    }
    occupancy_file_ << ros::Time::now().toSec() << "," <<  1 - (double)(nodes_to_visit_.size())/nodes_.size() << std::endl;
}

void FloorplanGraph::broadcastTransform(Node nextNode)
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "optimal_trajectory";
    
    
    transformStamped.transform.translation.x = nextNode.x;
    transformStamped.transform.translation.y = nextNode.y;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    broadcaster_.sendTransform(transformStamped);
}

bool FloorplanGraph::isEdgeClear(Node start, Node goal)
{
    // Bresenham's line algorithm
    int x_start = start.j;
    int y_start = start.i;

    int x_goal = goal.j;
    int y_goal = goal.i;

    int dx = std::abs(x_goal - x_start);
    int dy = std::abs(y_goal - y_start);

    int sx = x_start < x_goal ? 1 : -1;
    int sy = y_start < y_goal ? 1 : -1;

    int error = dx - dy;

    while(x_start != x_goal || y_start != y_goal)
    {
        if(occupancyGrid_(y_start,x_start) == 1)
        {
            return false;
        }

        int e2 = 2 * error;
        if(e2 > -dy)
        {
            error -= dy;
            x_start += sx;
        }
        if(e2 < dx)
        {
            error += dx;
            y_start += sy;
        }
    }

    return true;
}

bool FloorplanGraph::neighborsOccupied(int i, int j, int radius)
{
    for(int k = i - radius; k <= i + radius; k++)
    {
        for(int l = j - radius; l <= j + radius; l++)
        {
            if(k >= 0 && k < occupancyGrid_.rows() && l >= 0 && l < occupancyGrid_.cols())
            {
                if(occupancyGrid_(k,l) == 1)
                    return true;
            }
        }
    }
    return false;
}

void FloorplanGraph::buildOccupancyGrid(std::vector<std::vector<cv::Point>> contours)
{
    for (int i = 0; i < map_.rows; i++)
    {
        std::vector<int> row;
        for (int j = 0; j < map_.cols; j++)
        {
            int occupied = 0;

            // out of bounds
            std::pair<double,double> position = transformGridToMap(i, j);
            if(position.first >= x_max_ || position.first <= x_min_ || position.second >= y_max_ || position.second <= y_min_)
            {
                occupancyGrid_(i,j) = 1;
            }

            for(auto &contour : contours)
            {
                // if(cv::pointPolygonTest(contour, cv::Point2f(j, i), false) == 0){
                //     occupied = 1;
                //     break;
                // }
                for(auto &point : contour)
                {
                    if (std::abs(point.x - j) <= occupancy_radius_ && std::abs(point.y - i) <= occupancy_radius_)
                    {
                        occupied = 1;
                        break;
                    }
                    
                    
                }
            }
            occupancyGrid_(i,j) = occupied;
           
        }
       
        //occupancyGrid_.push_back(row);
    }
}


void FloorplanGraph::sampleNodes_random()
{
    std::ofstream node_file(node_log_file_);
    node_file <<  0 << " " << 0 << std::endl;
  
    for(int i = 0; i < num_samples_; i++)
    {
        int x = rand() % map_.cols;
        int y = rand() % map_.rows;
        if(occupancyGrid_(y,x) == 0 && !neighborsOccupied(y,x,skip_distance_)){
            std::pair<double,double> point = transformGridToMap(y, x);
            Node node = {point.first, point.second, y, x};
            node_file << node.x << " " << node.y << std::endl;
            nodes_.push_back(node);
        } 
    }

    node_file.close();

}

void FloorplanGraph::sampleNodes_uniform()
{
    std::ofstream node_file(node_log_file_);
    node_file <<  0 << " " << 0 << std::endl;

    int stepX = map_.rows / std::sqrt(num_samples_);
    int stepY = map_.cols / std::sqrt(num_samples_);
    for(int i = 1; i < map_.rows; i += stepX)
    {
        for(int j = 1; j < map_.cols; j += stepY)
        {

            if(occupancyGrid_(i,j) == 0 && !neighborsOccupied(i,j,skip_distance_)){
                std::pair<double,double> point = transformGridToMap(i, j);
                Node node = {point.first, point.second, i, j};
                node_file << node.x << " " << node.y << std::endl;
                nodes_.push_back(node);
            }
        }
    }

    node_file.close();
}

void FloorplanGraph::buildGraph()
{
    ROS_INFO("\n******************** Started Building Graph ********************\n");

    cv::resize(map_, map_, cv::Size(image_width_, image_height_));

    // Convert the image to grayscale
    cv::Mat imgray;
    cv::cvtColor(map_, imgray, cv::COLOR_BGR2GRAY);

    // Apply thresholding
    cv::Mat thresh;
    cv::threshold(imgray, thresh, 127, 255, 0);

    std::vector<std::vector<cv::Point>> contours;
    // See which makes more sense in retrieving segements
    //cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    ROS_DEBUG("Number of contours:  %d", contours.size());
    // Build occupancy grid
    occupancyGrid_ = Eigen::MatrixXi::Zero(map_.rows, map_.cols);

    buildOccupancyGrid(contours);

    ROS_DEBUG("Occupancy grid size:  %d x %d ", occupancyGrid_.rows(), occupancyGrid_.cols());
    
    // Add starting node
    std::pair<int,int> start_point = transformMapToGrid(start_x_, start_y_);
    Node start_node = {start_x_, start_y_, start_point.second, start_point.first};
    nodes_.push_back(start_node);

    // Create Node list
    if(sample_uniform_)
    {
        sampleNodes_uniform();
    }
    else
    {
        sampleNodes_random();
    }

    std::cout << "Number of nodes: " << nodes_.size() << std::endl;
  

    weightedEdgeMatrix_ = Eigen::MatrixXd::Zero(nodes_.size(), nodes_.size());
    // Fill Edge Matrix 
    // TODO: (ehosko) Matrix symmeric - only need to fill half
    for(int k = 0; k < nodes_.size(); k++)
    {
        for(int l = 0; l < nodes_.size(); l++){
            if(k != l)
            {
                double dist = nodes_[k].getDistance(nodes_[l]);

                if(isEdgeClear(nodes_[k],nodes_[l])){
                    weightedEdgeMatrix_(k,l) = std::round(dist * 10);
                }
                else{

                    if(contours_traversable_){
                        // Contours are traversable but add extra cost factor to adjust to height
                        double dist = nodes_[k].getDistance(nodes_[l]);
                        weightedEdgeMatrix_(k,l) = std::round(dist * 10) * contour_cost_factor_;
                    }
                    else{
                        weightedEdgeMatrix_(k,l) = 10e6;
                    }
                }
            }
        }
    }


    // Solve TSP problem
    TSPSolver tspSolver(lkh_executable_);
    tspSolver.initTSPSolver(weightedEdgeMatrix_);

    // starting node is 0
    double cost = tspSolver.solveTSP(tour_,0);

    
    std::ofstream opt_path_file(opt_path_log_file_);
    for(int i = 0; i < tour_->size() - 1; i++)
    {
        int idx = (*tour_)[i];
        int idx_next = (*tour_)[i+1];
        opt_path_file << i << " " << idx << " " << nodes_[idx].x << " " << nodes_[idx].y <<  " " << weightedEdgeMatrix_(idx,idx_next) << std::endl;
    }
    opt_path_file.close();

    if(cost > 0 && !(tour_->empty())){
        std_msgs::String msg;
   
        std::stringstream ss;
        ss << "Optimal Trajectory sucessfully computed!";
        msg.data = ss.str();
        ready_pub_.publish(msg);
    }

    next_node_counter_ = 0;
    nodes_to_visit_ = *tour_;
}

std::pair<double,double> FloorplanGraph::transformGridToMap(int i, int j)
{
    double x = (j * ((float)experiment_width_/image_width_) + x_min_);
    double y = -(i * ((float)experiment_height_/image_height_) - y_max_);

    return std::make_pair(x,y);
}

std::pair<int,int> FloorplanGraph::transformMapToGrid(double x, double y)
{
    int i = (int)(-y + y_max_)/ ((float)experiment_height_/image_height_);
    int j = (int)(x - x_min_ )/ ((float)experiment_width_/image_width_);

    return std::make_pair(i,j);
}
#include "../../include/referenceTrajectory/floorplan_graph.h"


FloorplanGraph::FloorplanGraph()
{
    // Constructor
}

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
    nh.getParam("floorplan_node/threshold", threshold_);

    nh.getParam("floorplan_node/skip_distance", skip_distance_);

    image_height_ = experiment_height_ / resolution_;
    image_width_ = experiment_width_ / resolution_;

    buildGraph();
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

void FloorplanGraph::buildGraph()
{
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

    std::cout << "Number of contours: " << contours.size() << std::endl;
    // Build occupancy grid
    occupancyGrid_ = Eigen::MatrixXi::Zero(map_.rows, map_.cols);

    std::cout << map_.rows << " " << map_.cols << std::endl;

    for (int i = 0; i < map_.rows; i++)
    {
        std::vector<int> row;
        for (int j = 0; j < map_.cols; j++)
        {
            int occupied = 0;
            for(auto &contour : contours)
            {
                if(cv::pointPolygonTest(contour, cv::Point2f(j, i), false) == 0){
                    occupied = 1;
                    break;
                }
            }
            occupancyGrid_(i,j) = occupied;
           
        }
       
        //occupancyGrid_.push_back(row);
    }
    std::cout << "Occupancy Grid: " <<  occupancyGrid_.rows() << " " <<occupancyGrid_.cols() << std::endl;

    for(int i = 0; i < num_samples_; i++)
    {
        int x = rand() % map_.cols;
        int y = rand() % map_.rows;
        if(occupancyGrid_(y,x) == 0 && !neighborsOccupied(y,x,skip_distance_)){
            std::pair<double,double> point = transformCoordinate(y, x);
            Node node = {point.first, point.second, y, x};
            nodes_.push_back(node);
        } 
    }

    std::cout << "Number of nodes: " << nodes_.size() << std::endl;

    weightedEdgeMatrix_ = Eigen::MatrixXd::Zero(nodes_.size(), nodes_.size());
    for(int k = 0; k < nodes_.size(); k++)
    {
        for(int l = 0; l < nodes_.size(); l++)
        {
            if(k != l)
            {
                double distance = sqrt(pow(nodes_[k].x - nodes_[l].x, 2) + pow(nodes_[k].y - nodes_[l].y, 2));
                if(distance < threshold_){
                    std::cout << "A* from " << nodes_[k].x << "'" << nodes_[k].y << " to " << nodes_[l].x << "'" << nodes_[l].y << std::endl;
                    weightedEdgeMatrix_(k,l) = aStar(nodes_[k], nodes_[l]);
                    std::cout << "Distance: " << weightedEdgeMatrix_(k,l) << std::endl;
                }
                else
                    weightedEdgeMatrix_(k,l) = -1;
            }
        }
    }

    std::cout << "Weighted Edge Matrix: " << weightedEdgeMatrix_ << std::endl;
}

std::pair<double,double> FloorplanGraph::transformCoordinate(int i, int j)
{
    double x = (j * ((float)experiment_width_/image_width_) - 0.5f*experiment_width_);
    double y = -(i * ((float)experiment_height_/image_height_) - 0.5f*experiment_height_);

    return std::make_pair(x,y);
}

struct NodeStar
{
    Node node;
    double f;
    double g;
    double h;
    NodeStar* parent;

    NodeStar(Node n, double g, double h, double f, NodeStar* p)
    {
        node = n;
        this->g = g;
        this->h = h;
        this->f = f;
        parent = p;
    }

    bool traversable(Eigen::MatrixXi occupancyGrid)
    {
        if(occupancyGrid(node.i,node.j) == 0)
            return true;
        else
            return false;
    }
};

struct GreaterNodeStar
{
    bool operator()(const NodeStar* n1, const NodeStar* n2)
    {
        return n1->f > n2->f;
    }
};

struct PointHash
{
    float operator()(const Node& p) const
    {
        return std::hash<float>()(p.x) ^ std::hash<float>()(p.y);
    }
};


double getDistance(Node n1, Node n2)
{
    return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
}

double FloorplanGraph::aStar(Node start, Node goal)
{
    std::priority_queue<NodeStar*,std::vector<NodeStar*>,GreaterNodeStar> openSet;
    std::unordered_map<Node, NodeStar*,PointHash> closedSet;

    // Create start node
    double h = getDistance(start, goal);
    NodeStar* startNode = new NodeStar(start,0,h,h,nullptr);

    // Add start node to openSet
    openSet.push(startNode);
    closedSet[start] = startNode;

    while(!openSet.empty())
    {
        // Get node with lowest f value
        NodeStar* current = openSet.top();
        Node current_node = current->node;
        openSet.pop();

        // Check if current node is goal
        if(current_node.i == goal.i && current_node.j == goal.j)
        {
            return current->g;
        }

        for(int i = current_node.i - 1; i <= current_node.i + 1; i += 1)
        {
            for(int j = current_node.j - 1; j <= current_node.j + 1; j += 1)
            {
                //std::cout << "Neighbor: " << i << "," << j << std::endl;

                // Skip current node
                if(i == current_node.i && j == current_node.j)
                {
                    continue;
                }
                // Check if neighbor is within FoV
                if (i < 0 || i >= static_cast<int>(occupancyGrid_.rows())|| j < 0 || j >= static_cast<int>(occupancyGrid_.cols()))
                {
                    continue;
                }

                std::pair<double,double> point = transformCoordinate(i, j);
                Node neighbor_node = {point.first,point.second,i,j};
                
                double tenetative_g = current->g + getDistance(current->node, neighbor_node);
                double tenetative_h = getDistance(neighbor_node, goal);
                double tenetative_f = tenetative_g + tenetative_h;
                if(!closedSet.count(neighbor_node) || tenetative_f < closedSet[neighbor_node]->f)
                {
                    // Create new node
                    //double h = getDistance(neighbor, goal);
                    NodeStar* neighbor = new NodeStar(neighbor_node, tenetative_g, tenetative_h, tenetative_f, current);

                    // Check if neighbor is traversable
                    if(neighbor->traversable(occupancyGrid_))
                    {
                        openSet.push(neighbor);
                        closedSet[neighbor_node] = neighbor;

                        //std::cout << "Pushed Neighbor: " << i << "," << j << std::endl;
                    }
                }
            }
        }
    }

    return -1;
}

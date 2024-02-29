#include "../../../include/localization/utils/aStar.h"

#include <stdlib.h>

struct Node
{
    cv::Point2f point;
    double g;
    double h;
    double f;
    Node* parent;

    Node(cv::Point2f p, double g, double h, double f, Node* parent)
    {
        this->point = p;
        this->g = g;
        this->h = h;
        this->f = f;
        this->parent = parent;
    }

    Node(cv::Point2f p)
    {
        this->point = p;
        this->g = std::numeric_limits<double>::infinity();
        this->h = std::numeric_limits<double>::infinity();
        this->f = std::numeric_limits<double>::infinity();
        this->parent = nullptr;
    }

    bool traversable(const std::vector<cv::Point2f>& segments,Eigen::Vector3d pos, double eps = 0.05)
    {
        bool trav = false;
        //Check if point is traversable
        for(int i = 0; i < segments.size(); i++)
        {
            // Check if point is on segment
            if(std::abs(segments[i].x - point.x) <= (eps * 0.1) && std::abs(segments[i].y - point.y) <= (eps *0.1))
            {
                return true;
            }
            else
            {
                // Check if point is within distance of segment

                cv::Point2f point = segments[i];
                double dist_segment = getDistance(point, this->point);
                double dist_robot = getDistance(cv::Point2f(pos(0),pos(1)), this->point);
                if(dist_segment + eps > dist_robot)
                {
                    trav = trav || true;
                }
            }
           
        }
        return trav;
    }
};


struct GreaterNode
{
    bool operator()(const Node* n1, const Node* n2)
    {
        return n1->f > n2->f;
    }
};

struct PointHash
{
    float operator()(const cv::Point2f& p) const
    {
        return std::hash<float>()(p.x) ^ std::hash<float>()(p.y);
    }
};

double getDistance(cv::Point2f p1, cv::Point2f p2){

    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}


bool aStar(cv::Point2f start, cv::Point2f goal, std::vector<cv::Point2f>& path, std::vector<cv::Point2f> segments,Eigen::Vector3d pos,double x_min, double x_max, double y_min, double y_max,double resolution, double epsilon = 0.01)
{
    // Astar algorithm to find path from start to goal
    // Return true if path is found, false otherwise
    std::priority_queue<Node*,std::vector<Node*>,GreaterNode> openSet;
    std::unordered_map<cv::Point2f, Node*,PointHash> closedSet;

    // Create start node
    double h = getDistance(start, goal);
    Node* startNode = new Node(start,0,h,h,nullptr);

    // Add start node to openSet
    openSet.push(startNode);
    closedSet[start] = startNode;

    while(!openSet.empty())
    {
        // Get node with lowest f value
        Node* current = openSet.top();
        openSet.pop();

        // Check if current node is goal
        if(std::abs((current->point).x - goal.x) < epsilon && std::abs((current->point).y - goal.y) < epsilon)
        {
            // Reconstruct path
            while(current != nullptr)
            {
                path.push_back(current->point);
                current = current->parent;
            }
            return true;
        }

        // Loop through neighbors
        double x = current->point.x;
        double y = current->point.y;
        for(double i = x - resolution; i <= x + resolution; i += resolution)
        {
            for(double j = y - resolution; j <= y + resolution; j += resolution)
            {
                // Skip current node
                if(std::abs(x-i) < std::numeric_limits<double>::epsilon() && std::abs(y-j) < std::numeric_limits<double>::epsilon())
                {
                    continue;
                }
                // Check if neighbor is within FoV
                if (i < x_min || i > x_max || j < y_min || j > y_max)
                {
                    continue;
                }

                cv::Point2f neighbor(i,j);
                
                double tenetative_g = current->g + getDistance(current->point, neighbor);
                double tenetative_h = getDistance(neighbor, goal);
                double tenetative_f = tenetative_g + tenetative_h;
                if(!closedSet.count(neighbor) || tenetative_f < closedSet[neighbor]->f)
                {
                    // Create new node
                    //double h = getDistance(neighbor, goal);
                    Node* neighborNode = new Node(neighbor, tenetative_g, tenetative_h, tenetative_f, current);

                    // Check if neighbor is traversable
                    if(neighborNode->traversable(segments,pos))
                    {
                        openSet.push(neighborNode);
                        closedSet[neighbor] = neighborNode;

                    }
                }
            }
        }
    }

    return false;

}
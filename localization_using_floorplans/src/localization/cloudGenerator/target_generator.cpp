#include "../../../include/localization/cloudGenerator/target_generator.h"
#include <stdlib.h>


TargetGenerator::TargetGenerator()
{
    // Constructor
}

TargetGenerator::~TargetGenerator()
{
    // Destructor
}

void TargetGenerator::generateTargetCloud()
{
    // Generate target cloud
    std::cout << "Generating target cloud..." << std::endl;
    std::cout << "Getting Contours..." << std::endl;
    computeContours();
    std::cout << "Dividing Contours..." << std::endl;
    std::vector<cv::Point> candidates = divideContours();
    std::cout << "Number of candidates: " << candidates.size() << std::endl;
    std::cout << "Filter Points..." << std::endl;
    filterAccesiblePoints(candidates);

    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const cv::Point& point : candidates)
    {
        pcl::PointXYZ pcl_point;
        pcl_point.x = static_cast<float>(point.x);
        pcl_point.y = static_cast<float>(point.y);
        pcl_point.z = 0.0;  // Assuming 2D points with z=0

        targetCloud->points.push_back(pcl_point);
    }

    targetCloud->width = targetCloud->points.size();
    targetCloud->height = 1;

    _targetCloud = *targetCloud;
    std::cout << "Target cloud generated of size " << _targetCloud.width << std::endl;

    // Save target cloud to file
    pcl::PCDWriter w;
    std::string file_name = "/home/michbaum/ElisaSemesterProject/data/pcdData/targetCloud.pcd";
    w.write<pcl::PointXYZ> (file_name, _targetCloud, false);
}

void TargetGenerator::computeContours()
{
    // Convert the image to grayscale
    cv::Mat imgray;
    cv::cvtColor(image, imgray, cv::COLOR_BGR2GRAY);

    // Apply thresholding
    cv::Mat thresh;
    cv::threshold(imgray, thresh, 127, 255, 0);

    // See which makes more sense in retrieving segements
    cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    //cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
}

// Divide contours into segments < l_max
std::vector<cv::Point> TargetGenerator::divideContours()
{
    std::vector<cv::Point> candidatePoints;

    int num_contours = contours.size();
    for(int i = 0; i < num_contours; i++)
    {
        int j = 0;
        for(j = 0; j < contours[i].size() - 1; j++)
        {
            // Check if distance between points is greater than l_max
            // If so, add new point to contours[i] at index j
            double dist = getDistance(contours[i][j], contours[i][j+1]);
            if(dist > l_max)
            {
                // Divide contour into segments of length l_max
                // Calculate the number of segments needed
                int num_segments = std::ceil(dist / l_max);

                // Calculate the segment length
                int segment_length = std::ceil(dist / num_segments);

                cv::Point startPoint = contours[i][j];
                cv::Point diff = contours[i][j+1] - contours[i][j];

                candidatePoints.push_back(startPoint);
                // Divide contour into segments of length l_max
                for(int k = 1; k < num_segments - 1; k++)
                {
                    // Calculate the position of the new point
                    cv::Point new_point = startPoint + diff * (segment_length * k / dist);

                    // Insert the new point at the appropriate index in the contour
                    candidatePoints.push_back(new_point);
                }
                candidatePoints.push_back(contours[i][j+1]);
            }else
            {
                // Add point to candidatePoints
                candidatePoints.push_back(contours[i][j]);
            } 
        }
        candidatePoints.push_back(contours[i][j]);
    }

    return candidatePoints;
}

void TargetGenerator::filterAccesiblePoints(const std::vector<cv::Point>& candidates)
{
    
    std::vector<cv::Point> accessiblePoints;
    // Filter out points that are not accessible

    // Generate a matrix of cv::Points of floorplan that lies within the FoV of the camera

    // Filter out points that are not in FoV
    // Camera Model: getVisibleVoxels()

    std:: cout << "Number of candidates: " << candidates.size() << std::endl;
    std::vector<cv::Point> foVPoints(&candidates[0], &candidates[0] + candidates.size() / 10);
    std:: cout << "Number of FoV candidates: " << foVPoints.size() << std::endl;

    // Filter out points that cannot be connected by A*
    for(int i = 0; i < foVPoints.size() - 1; i++)
    {
        std::cout << "A* star iteration:" << i << ", with points: " << foVPoints[i] << "&" << foVPoints[i + 1] << std::endl;


        std::vector<cv::Point> path;
        bool success = aStar(foVPoints[i], foVPoints[i + 1], path);
        std::cout << "Success: " << success << std::endl;
        if(success)
        {
            for(int j = 0; j < path.size(); j++)
            {
                accessiblePoints.push_back(path[j]);
            }
        }
    
    }
    
}

double TargetGenerator::getDistance(cv::Point p1, cv::Point p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

std::vector<cv::Point> TargetGenerator::getSegments()
{
    computeContours();
    return divideContours();
}

struct Node
{
    cv::Point point;
    double g;
    double h;
    double f;
    Node* parent;

    Node(cv::Point p, double g, double h, double f, Node* parent)
    {
        this->point = p;
        this->g = g;
        this->h = h;
        this->f = f;
        this->parent = parent;
    }

    Node(cv::Point p)
    {
        this->point = p;
        this->g = std::numeric_limits<double>::infinity();
        this->h = std::numeric_limits<double>::infinity();
        this->f = std::numeric_limits<double>::infinity();
        this->parent = nullptr;
    }

    bool traversable()
    {
        // Check if point is traversable
        // TODO (eho): Implement
        int random = rand() % 100;
        if(random < 30)
        {
            return false;
        }
        return true;
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
    std::size_t operator()(const cv::Point& p) const
    {
        return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);
    }
};


bool TargetGenerator::aStar(cv::Point start, cv::Point goal, std::vector<cv::Point>& path)
{
    // Boundaries
    int x_min = 0;
    int x_max = 700;
    int y_min = 0;
    int y_max = 700;

    // Astar algorithm to find path from start to goal
    // Return true if path is found, false otherwise
    std::priority_queue<Node*,std::vector<Node*>,GreaterNode> openSet;
    std::unordered_map<cv::Point, Node*,PointHash> closedSet;

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
        if(current->point == goal)
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
        int x = current->point.x;
        int y = current->point.y;
        for(int i = x - 1; i <= x + 1; i++)
        {
            for(int j = y - 1; j <= y + 1; j++)
            {
                // Skip current node
                if(i == x && j == y)
                {
                    continue;
                }
                // Check if neighbor is within FoV
                if (i < x_min || i > x_max || j < y_min || j > y_max)
                {
                    continue;
                }

                cv::Point neighbor(i,j);
                
                double tenetative_g = current->g + getDistance(current->point, neighbor);
                if(!closedSet.count(neighbor) || tenetative_g < closedSet[neighbor]->g)
                {
                    // Create new node
                    double h = getDistance(neighbor, goal);
                    Node* neighborNode = new Node(neighbor, tenetative_g, h, tenetative_g + h, current);

                    // Check if neighbor is traversable
                    if(neighborNode->traversable())
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
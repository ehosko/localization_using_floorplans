#include "../../include/referenceTrajectory/tsp_solver.h"
#include <iostream>

TSPSolver::TSPSolver()
{
    //ctor
}

TSPSolver::~TSPSolver()
{
    //dtor
}

void TSPSolver::initTSPSolver(Eigen::MatrixXd weightedAdjacenyMatrix)
{
    tsp_file_ = "problem.tsp";
    tsp_tour_ = "problem.tour";

    CreateTSPFile(weightedAdjacenyMatrix);
}

void TSPSolver::solveTSP(std::vector<int>& path)
{
    std::string command = concorde_executable_ + " -x -w -o " + tsp_tour_ + " " + tsp_file_;
    system(command.c_str());

    ReadTourFile();
}

void TSPSolver::CreateTSPFile(Eigen::MatrixXd weightedAdjacenyMatrix)
{
    std::ofstream file(tsp_file_);
    file << "NAME: Floorplan" << std::endl;
    file << "TYPE: TSP" << std::endl;
    file << "DIMENSION: " << weightedAdjacenyMatrix.rows() << std::endl;
    file << "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl;
    file << "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl;
    file << "EDGE_WEIGHT_SECTION" << std::endl;

    for(int i = 0; i < weightedAdjacenyMatrix.rows(); i++)
    {
        for(int j = 0; j < weightedAdjacenyMatrix.cols(); j++)
        {
            file << weightedAdjacenyMatrix(i,j) << " ";
        }
        file << std::endl;
    }
    file << "EOF" << std::endl;
    file.close();
}

void TSPSolver::ReadTourFile()
{
    std::ifstream file(tsp_tour_);
    std::string line;
    std::string tour;

    // Second line stores the path
    std::getline(file, line);
    std::getline(file, line);
    tour = line;
    // while(std::getline(file, line))
    // {
    //     if(line.find("TOUR_SECTION") != std::string::npos)
    //     {
    //         std::getline(file, line);
    //         tour = line;
    //     }
    // }
    file.close();

    std::vector<int> path;
    std::stringstream ss(tour);
    int i;
    while(ss >> i)
    {
        path.push_back(i);
        if(ss.peek() == ',')
            ss.ignore();
    }
}

void TSPSolver::rotatePath(std::vector<int>& path, int start){
    std::rotate(path.begin(), path.begin() + start, path.end());
}


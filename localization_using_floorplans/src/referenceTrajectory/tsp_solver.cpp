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
    tsp_params_ = "problem.par";

    std::ofstream file(tsp_params_);
    file << "PROBLEM_FILE = " << tsp_file_ << std::endl;
    file << "OUTPUT_TOUR_FILE = " << tsp_tour_ << std::endl;

    CreateTSPFile(weightedAdjacenyMatrix);
}

void TSPSolver::solveTSP(std::vector<int>* path,int start)
{
    std::string command = lkh_executable_ + " " + tsp_params_;
    system(command.c_str());

    ReadTourFile(path);
    //rotatePath(path, start);
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
            file << (int)(weightedAdjacenyMatrix(i,j)) << " ";
        }
        file << std::endl;
    }
    file << "EOF" << std::endl;
    file.close();
}

void TSPSolver::ReadTourFile(std::vector<int>* path)
{
    std::ifstream file(tsp_tour_);
    std::string line;
    std::string tour;

    while(std::getline(file, line))
    {
        if(line.find("TOUR_SECTION") != std::string::npos)
        {
            std::getline(file, line);
            int idx = std::stoi(line, nullptr, 10);

            while(idx != -1)
            {
                // -1 because the tour starts at 1
                path->push_back(idx - 1);
                std::getline(file, line);
                idx = std::stoi(line, nullptr, 10);
            }
        }
    }
    file.close();


}

void TSPSolver::rotatePath(std::vector<int>* path, int start){
    std::rotate(path->begin(), path->begin() + start, path->end());
}


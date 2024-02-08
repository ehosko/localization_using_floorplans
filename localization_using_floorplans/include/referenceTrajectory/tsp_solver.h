#ifndef TSP_SOLVER_H
#define TSP_SOLVER_H

#include <vector>
#include <eigen3/Eigen/Geometry>

#include <fstream>

class TSPSolver
{
public:
    TSPSolver();
    TSPSolver(std::string executable): lkh_executable_(executable) {};
    ~TSPSolver();

    void initTSPSolver(Eigen::MatrixXd weightedAdjacenyMatrix);

    void solveTSP(std::vector<int>* path, int start);
    
private:

    void CreateTSPFile(Eigen::MatrixXd weightedAdjacenyMatrix);
    void ReadTourFile(std::vector<int>* path);

    void rotatePath(std::vector<int>* path, int start);

    std::string lkh_executable_;

    std::string tsp_file_ = "tsp_file.tsp";
    std::string tsp_tour_ = "tsp_file.tour";
    std::string tsp_params_ = "tsp_file.par";

};

#endif // TSP_SOLVER_H
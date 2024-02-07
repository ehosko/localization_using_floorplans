#ifndef TSP_SOLVER_H
#define TSP_SOLVER_H

#include <vector>
#include <eigen3/Eigen/Geometry>

#include <fstream>

class TSPSolver
{
public:
    TSPSolver();
    TSPSolver(std::string concorde_executable): concorde_executable_(concorde_executable) {};
    ~TSPSolver();

    void initTSPSolver(Eigen::MatrixXd weightedAdjacenyMatrix);

    void solveTSP(std::vector<int>& path);
    
private:

    void CreateTSPFile(Eigen::MatrixXd weightedAdjacenyMatrix);
    void ReadTourFile();

    void rotatePath(std::vector<int>& path, int start);

    std::string concorde_executable_;

    std::string tsp_file_ = "tsp_file.tsp";
    std::string tsp_tour_ = "tsp_file.tour";

};

#endif // TSP_SOLVER_H
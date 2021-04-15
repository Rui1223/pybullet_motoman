#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include "Graph.hpp"
#include "AstarSolver.hpp"
#include "Timer.hpp"

int main(int argc, char** argv)
{
    std::string task_name = std::string(argv[1]);
    std::string armType = std::string(argv[2]);
    int nsamples = atoi(argv[3]);
    std::string method = std::string(argv[4]);
    std::string samples_file = "../roadmaps/samples_" + armType + ".txt";
    std::string connections_file = "../roadmaps/connections_" + armType + ".txt";
    std::string task_file = "../src/" + task_name + ".txt";

    // construct the graph given these files
    Timer t;
    Graph_t g(samples_file, connections_file, task_file, nsamples);
    std::cout << "Time to import the graph for " << g.getnNodes() << " nodes: " << t.elapsed() << "\n";
    
    // call the planning method
    if (method == "shortestPath") {
        AstarSolver_t astar_solver(g, g.getStart(), g.getGoal());
        astar_solver.Astar_search(g);
        if (astar_solver.getFailure() == false) {
            astar_solver.printAll();
            std::cout << "\n";
        }
        // write the trajectory
        // std::string task_trajectory_file = "../src/" + task_name + "_traj.txt";
        // astar_solver.writeTrajectory(task_trajectory_file);

        // write the path directly
        std::string task_path_file = "../src/" + task_name + "_traj.txt";
        astar_solver.writePath(task_path_file);

    }


    return 0;

}
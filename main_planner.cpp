#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

#include "Graph.hpp"
#include "AstarSolver.hpp"

int main(int argc, char** argv)
{
    std::string scene_index = std::string(argv[1]);
    std::string task_name = std::string(argv[2]);
    std::string handType = std::string(argv[3]);
    int nsamples = atoi(argv[4]);
    std::string method = std::string(argv[5]);
    std::string task_file = "./roadmaps/" + scene_index + "/" + task_name + ".txt";
    std::string samples_file = "./roadmaps/" + scene_index + "/samples_" + handType + ".txt";
    std::string connections_file = "./roadmaps/" + scene_index + "/connections_" + handType + ".txt";


    // construct the graph given these files
    Graph_t g(samples_file, connections_file, task_file, nsamples);
    
    // call the planning method
    if (method == "shortestPath") {
        AstarSolver_t astar_solver(g, g.getStart(), g.getGoal());
        astar_solver.Astar_search(g);
        if (astar_solver.getFailure() == false) {
            astar_solver.printAll();            
        }
        // write the trajectory
        std::string task_trajectory_file = "./roadmaps/" + scene_index + "/" + task_name + "_traj.txt";
        astar_solver.writeTrajectory(task_trajectory_file);

    }


    return 0;

}
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <ros/package.h>
#include "pybullet_motoman/AstarPathFinding.h"

#include "Graph.hpp"
#include "AstarSolver.hpp"
#include "Timer.hpp"

// struct AstarResult
// {
//     bool searchSuccess;
//     std::vector<int> path;
// };

class Planner_t {

public:

    Graph_t m_left_g;
    Graph_t m_right_g;
    AstarSolver_t m_astar_solver;

    // constructor
    Planner_t() {}
    Planner_t(std::string left_samples_file, std::string left_connections_file, 
                std::string right_samples_file, std::string right_connections_file) {

        m_left_g.constructGraph(left_samples_file, left_connections_file);
        m_right_g.constructGraph(right_samples_file, right_connections_file);

    }

    bool astarSolverCallback(
        pybullet_motoman::AstarPathFinding::Request &req,
        pybullet_motoman::AstarPathFinding::Response &resp
        ) {
        if (req.armType == "Left"){
            if (m_astar_solver.getQueryIdx() != req.query_idx) {
                // this is a new query, let's set the new query
                m_astar_solver.setPlanningQuery(m_left_g, req.query_idx, 
                    req.start_idx, req.goal_idx, req.start_config, req.goal_config,
                    req.start_neighbors_idx, req.goal_neighbors_idx,
                    req.start_neighbors_cost, req.goal_neighbors_cost,
                    req.violated_edges);
            }
            m_left_g.modifyEdge(req.violated_edges, req.query_idx);
            m_astar_solver.prepareToSearch(m_left_g);
            m_astar_solver.Astar_search(m_left_g);
        }
        else {
            if (m_astar_solver.getQueryIdx() != req.query_idx) {
                // this is a new query, let's set the new query
                m_astar_solver.setPlanningQuery(m_right_g, req.query_idx, 
                    req.start_idx, req.goal_idx, req.start_config, req.goal_config,
                    req.start_neighbors_idx, req.goal_neighbors_idx,
                    req.start_neighbors_cost, req.goal_neighbors_cost,
                    req.violated_edges);
            }
            m_right_g.modifyEdge(req.violated_edges, req.query_idx);
            m_astar_solver.prepareToSearch(m_right_g);
            m_astar_solver.Astar_search(m_right_g);
        }

        // let's return the response after a search
        resp.searchSuccess = m_astar_solver.getSearchSuccessInfo();
        resp.path = m_astar_solver.getPath();
        if (resp.searchSuccess == true) {
            return true;
        }
        else {
            return false;
        }

    }

};

int main(int argc, char** argv)
{
    // initialize the ros node
    ros::init(argc, argv, "main_planner");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("initialize main_plannner_node");

    std::string package_path = ros::package::getPath("pybullet_motoman");
    Timer t;

    // load the roadmap for left and right arm
    std::string left_samples_file = package_path + "/roadmaps/samples_Left.txt";
    std::string left_connections_file = package_path + "/roadmaps/connections_Left.txt";
    std::string right_samples_file = package_path + "/roadmaps/samples_Right.txt";
    std::string right_connections_file = package_path + "/roadmaps/connections_Right.txt";
    // Graph_t left_g(left_samples_file, left_connections_file);
    // Graph_t right_g(right_samples_file, right_connections_file);
    Planner_t planner(left_samples_file, left_connections_file, right_samples_file, right_connections_file);

    // planner.printWrapper();
    std::cout << "time to load graph with " << planner.m_left_g.getnNodes() << " nodes is " << t.elapsed() << "\n";
    std::cout << "time to load graph with " << planner.m_right_g.getnNodes() << " nodes is " << t.elapsed() << "\n";

    // claim service the node provide (server)
    ros::ServiceServer server = nh.advertiseService("astar_path_finding", &Planner_t::astarSolverCallback, &planner);

    // Loop at 2Hz until the node is shut down
    // ros::Rate rate(2);
    while (ros::ok()) {
        ros::spin();
        // rate.sleep();
    }

    return 0;

}


//////////////// below is not used but just kept for legacy ////////////////

// bool astarSolverCallback(
//     pybullet_motoman::AstarPathFinding::Request &req,
//     pybullet_motoman::AstarPathFinding::Response &resp
//     ) {


//     // std::cout << "print info in c++\n";
//     // std::cout << "query_idx: " << req.query_idx << "\n";
//     // std::cout << "start_idx: " << req.start_idx << "\n";
//     // std::cout << "goal_idx: " << req.goal_idx << "\n";
//     // std::cout << "start_config: ";
//     // for (auto const &e : req.start_config) {
//     //     std::cout << e << " ";
//     // }
//     // std::cout << "\n";
//     // std::cout << "goal_config: ";
//     // for (auto const &e : req.goal_config) {
//     //     std::cout << e << " ";
//     // }
//     // std::cout << "\n";
//     // std::cout << "start_neighbors_idx: ";
//     // for (auto const &e : req.start_neighbors_idx) {
//     //     std::cout << e << " ";
//     // }
//     // std::cout << "\n";
//     // std::cout << "goal_neighbors_idx: ";
//     // for (auto const &e : req.goal_neighbors_idx) {
//     //     std::cout << e << " ";
//     // }
//     // std::cout << "\n";
//     // std::cout << "start_neighbors_cost: ";
//     // for (auto const &e : req.start_neighbors_cost) {
//     //     std::cout << e << " ";
//     // }
//     // std::cout << "\n";
//     // std::cout << "goal_neighbors_cost: ";
//     // for (auto const &e : req.goal_neighbors_cost) {
//     //     std::cout << e << " ";
//     // }
//     // std::cout << "\n";
//     // std::cout << "violated_edges: ";
//     // for (auto const &e : req.violated_edges) {
//     //     std::cout << "(" << e.idx1 << ", " << e.idx2 << ")" << "\t";
//     // }
//     // std::cout << "\n";
//     // std::cout << "armType: " << req.armType << "\n";

//     // resp.searchSuccess = true;
//     // resp.path = std::vector<int>{5000, 5001, 5, 7};

//     // std::cout << "***************\n";

//     // return true;

// // }
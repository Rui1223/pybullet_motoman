/* This hpp file declares A* search on a given graph
with specified start and goal */

#ifndef ASTARSOLVER_H
#define ASTARSOLVER_H

#include <vector>
#include <queue>
#include <cstring>
#include <fstream>

#include "Graph.hpp"

struct AstarNode_t
{
    int m_id;
    float m_g;
    AstarNode_t *m_parent;

    AstarNode_t(int id, float g, AstarNode_t *p) {
        m_id = id;
        m_g = g;
        m_parent = p;
    }
};

struct AstarNode_comparison
{
    bool operator()(const AstarNode_t* a, const AstarNode_t* b)
    {
        return (a->m_g) > (b->m_g);
    }
};

class AstarSolver_t
{
    std::vector<int> m_path;
    std::vector<std::vector<float>> m_trajectory;
    std::priority_queue<AstarNode_t*, std::vector<AstarNode_t*>, AstarNode_comparison> m_open;
    std::vector<AstarNode_t*> m_closed;
    std::vector<bool> m_expanded;
    std::vector<float> m_G;
    std::vector<float> m_H;

    int m_start;
    int m_goal;

    std::ofstream m_outFile_;
    bool m_isFailure;
    bool m_isPathSuccess;
    float m_pathCost;

public:
    // Constructor
    AstarSolver_t(Graph_t &g, int start, int goal);
    // destructor
    ~AstarSolver_t();

    void Astar_search(Graph_t &g);

    void computeH(Graph_t &g);
    float computeDist(std::vector<float> state1, std::vector<float> state2);
    void back_track_path();
    void pathToTrajectory(Graph_t &g);
    void writeTrajectory(std::string task_trajectory_file);

    // printer
    void print_path();
    void print_cost();
    void printAll();

    // getter
    bool getFailure() { return m_isFailure; }
    std::vector<std::vector<float>> getTrajectory() { return m_trajectory; }

};


#endif
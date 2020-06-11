/* This cpp file defines A* search on a given graph
with specified start and goal */

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "Graph.hpp"
#include "AstarSolver.hpp"

AstarSolver_t::AstarSolver_t(Graph_t &g, int start, int goal)
{
    // initialize the start & goal
    m_start = start;
    m_goal = goal;
    // essential elements for Astar search
    computeH(g); // heuristics
    m_G = std::vector<float>(g.getnNodes(), std::numeric_limits<float>::max());
    m_G[m_start] = 0.0;
    m_open.push(new AstarNode_t(m_start, m_G[m_start], nullptr));
    m_expanded = std::vector<bool>(g.getnNodes(), false);

    m_isFailure = false;
    
}

AstarSolver_t::~AstarSolver_t()
{
    while (!m_open.empty())
    {
        AstarNode_t* a1 = m_open.top();
        delete a1;
        m_open.pop();
    }
    for (auto &e : m_closed) { delete e; }
}

void AstarSolver_t::computeH(Graph_t &g)
{
    // loop through all nodes
    for (int i=0; i < g.getnNodes(); i++) {
        m_H.push_back(computeDist(g.getState(i), g.getGoalState()));
    }

}

float AstarSolver_t::computeDist(std::vector<float> state1, std::vector<float> state2)
{
    float temp_dist = 0.0;
    for (int k=0; k < state1.size(); k++) {
        temp_dist += pow(state1[k]-state2[k], 2);
    }
    temp_dist = sqrt(temp_dist);
    return temp_dist;
}

void AstarSolver_t::Astar_search(Graph_t &g)
{
    while (!m_open.empty()){
        AstarNode_t *current = m_open.top();
        m_open.pop();
        // Now check if the current node has been expanded
        if (m_expanded[current->m_id] == true) {
            // This node has been expanded with the lowest f value for its id
            // No need to put it into the closed list
            delete current;
            continue;
        }
        // std::cout << current->m_id << "\n";
        m_closed.push_back(current);
        m_expanded[current->m_id] = true;

        if (current->m_id == m_goal) {
            // the goal is found
            std::cout << "PATH FOUND\n";
            back_track_path(); // construct your path
            pathToTrajectory(g); // get the trajectory (a sequence of configurations)
            m_pathCost = current->m_g;

            return;
        }
        // get neighbors of the current node
        std::vector<int> neighbors = g.getNodeNeighbors(current->m_id);
        for (auto const &neighbor : neighbors)
        {
            // check if the neighbor node has been visited or extended before
            if ( m_expanded[neighbor] ) {continue;}
            if ( m_G[neighbor] > m_G[current->m_id] + g.getEdgeCost(current->m_id, neighbor) )
            {
                m_G[neighbor] = m_G[current->m_id] + g.getEdgeCost(current->m_id, neighbor);
                m_open.push(new AstarNode_t(neighbor, m_G[neighbor], current));
            }
        }
    }
    // You are reaching here since the open list is empty and the goal is not found
    std::cout << "The problem is not solvable. Search failed...\n\n";
    m_isFailure = true;
}

void AstarSolver_t::back_track_path()
{
    // start from the goal
    AstarNode_t *current = m_closed[m_closed.size()-1];
    while (current->m_id != m_start)
    {
        // keep backtracking the path until you reach the start
        m_path.push_back(current->m_id);
        current = current->m_parent;
    }
    // finally put the start into the path
    m_path.push_back(current->m_id);
}

void AstarSolver_t::pathToTrajectory(Graph_t &g)
{
    // start from the start
    for (int i=m_path.size()-1; i >=0; i--)
    {
        m_trajectory.push_back(g.getState(m_path[i]));

    }
    // // print the trajectory for checking purpose
    // std::cout << "The trajectory: \n";
    // for (auto const &t : m_trajectory)
    // {
    //  for (auto const &d : t)
    //  {
    //      std::cout << d << "   ";
    //  }
    //  std::cout << "\n";
    // }
}

void AstarSolver_t::writeTrajectory(std::string trajectory_file)
{
    m_outFile_.open(trajectory_file);
    if (m_outFile_.is_open())
    {
        // first write in failure indicator
        m_outFile_ << m_isFailure << "\n";
        if (m_isFailure == false) {
            for (auto const &t : m_trajectory)
            {
                for (auto const &d : t)
                {
                    m_outFile_ << d << " ";
                }
                m_outFile_ << "\n";
            }            
        }

    }
    m_outFile_.close();

}

void AstarSolver_t::printAll()
{
    print_path();
    print_cost();
}

void AstarSolver_t::print_path()
{
    // print the path for checking purpose
    std::cout << "path: \n";
    for (int i=m_path.size()-1; i >=0; i--)
    {
        std::cout << m_path[i] << " ";
    }
    // for (auto const &waypoint : m_path)
    // {
    //  std::cout << waypoint << " ";
    // }
    std::cout << "\n";
}

void AstarSolver_t::print_cost()
{
    std::cout << "cost: " << m_pathCost << "\n";
}
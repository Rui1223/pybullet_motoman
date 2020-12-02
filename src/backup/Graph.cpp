/* This cpp file defines a graph which is constructed 
from the roadmap built in robotic scenarios.*/

#include <vector>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <limits>
#include <typeinfo>
#include <cmath>

#include "Graph.hpp"

Graph_t::Graph_t(std::string samples_file, std::string connections_file, std::string task_file, int nsamples)
{
    m_nNodes = nsamples + 2;
    specify_nodeStates(samples_file);
    specify_neighborCosts(connections_file);
    connectStartAndGoal(task_file);


    // printStates();
    // printNeighbors();
    // printEdgeCosts();
}


float Graph_t::computeDist(std::vector<float> n1, std::vector<float> n2) {
    float temp_dist = 0.0;
    for (int j=0; j < n1.size(); j++) {
        temp_dist += pow(n1[j] - n2[j], 2);
    }
    temp_dist = sqrt(temp_dist);
    return temp_dist;
}



void Graph_t::connectStartAndGoal(std::string task_file)
{
    // specify start and goal
    m_start = m_nNodes - 2;
    m_goal = m_nNodes - 1;
    /// Now read in the task file to get start & goal information
    m_inFile_.open(task_file);
    if (!m_inFile_)
    {
        std::cerr << "Unable to open the task file\n";
        exit(1); // call system to stop     
    }

    int nline = 0;
    std::string temp_str;
    while (std::getline(m_inFile_, temp_str))
    {
        nline += 1;
        std::stringstream ss(temp_str);

        if (nline == 1 or nline == 2) {
            int temp_nodeIdx;
            ss >> temp_nodeIdx;
            std::vector<float> temp_d;
            float d;
            while (ss >> d)
            {
                temp_d.push_back(d);
            }
            m_nodeStates.push_back(temp_d);
        }
        else {
            // add connection information for the start and goal
            int temp_n1;
            int temp_n2;
            float temp_cost;
            ss >> temp_n1 >> temp_n2 >> temp_cost;
            m_nodeNeighbors[temp_n1].push_back(temp_n2);
            m_nodeNeighbors[temp_n2].push_back(temp_n1);
            m_edgeCosts[temp_n1][temp_n2] = temp_cost;
            m_edgeCosts[temp_n2][temp_n1] = temp_cost;

        }

    }
    m_inFile_.close();
    
    m_startNode = m_nodeStates[m_start];
    m_goalNode = m_nodeStates[m_goal];

}


void Graph_t::specify_neighborCosts(std::string connections_file)
{
    int iter = 0;
    // create empty neighbors and cost vector for each node
    while (iter != m_nNodes) {
        m_nodeNeighbors.push_back(std::vector<int>());
        m_edgeCosts.push_back(std::vector<float>(m_nNodes, std::numeric_limits<float>::max()));
        iter++;
    }
    // read in the connection file
    m_inFile_.open(connections_file);
    // Check that the file was opened successfully
    if (!m_inFile_)
    {
        std::cerr << "Unable to open the connections file\n";
        exit(1); // call system to stop
    }
    std::string temp_str;
    int temp_n1;
    int temp_n2;
    float temp_cost;
    while (std::getline(m_inFile_, temp_str))
    {
        std::stringstream ss(temp_str);
        ss >> temp_n1 >> temp_n2 >> temp_cost;
        m_nodeNeighbors[temp_n1].push_back(temp_n2);
        m_nodeNeighbors[temp_n2].push_back(temp_n1);
        m_edgeCosts[temp_n1][temp_n2] = temp_cost;
        m_edgeCosts[temp_n2][temp_n1] = temp_cost;
    }
    m_inFile_.close();

}   

void Graph_t::specify_nodeStates(std::string samples_file)
{
    // read in the samples file
    m_inFile_.open(samples_file);
    // Check that the file was opened successfully
    if (!m_inFile_)
    {
        std::cerr << "Unable to open the samples file\n";
        exit(1); // call system to stop
    }
    std::string temp_str;
    int temp_nodeIdx;
    float c;
    /// read through samples file to construct m_nodeStates
    while (std::getline(m_inFile_, temp_str))
    {
        std::stringstream ss(temp_str);
        ss >> temp_nodeIdx;
        std::vector<float> temp_v;
        while (ss >> c)
        {
            temp_v.push_back(c);
        }
        m_nodeStates.push_back(temp_v);
    }
    m_inFile_.close();

}


void Graph_t::printStates()
{
    for (auto const &state : m_nodeStates) {
        for (auto const &e : state) {
            std::cout << e << " ";
        }
        std::cout << "\n";
    }
}

void Graph_t::printNeighbors()
{
    for (auto const &neighbors : m_nodeNeighbors) {
        for (auto const &id : neighbors) {
            std::cout << id << " ";
        }
        std::cout << "\n";
    }
}

void Graph_t::printEdgeCosts()
{
    for (auto const &costs : m_edgeCosts) {
        for (auto const &value : costs) {
            std::cout << value << " ";
        }
        std::cout << "\n";
    }
}
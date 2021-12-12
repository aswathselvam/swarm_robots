/**
 * @file arena.h
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for arena elements
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */
#ifndef INCLUDE_SWARM_ROBOTS_ARENA_HPP_  //  NOLINT
#define INCLUDE_SWARM_ROBOTS_ARENA_HPP_

#include <std_msgs/String.h>

#include <vector>

#include "agent.hpp"
#include "state.hpp"

using std::vector;
class Arena {
   private:  //  NOLINT
    const int total_agents = 4;
    // TODO(kavya): change number to 20 - total nodes
    vector<Agent> agents;
    // vector<double, double> boundary;
    State origin;

   public:  //  NOLINT
    bool InitializeAgents();
    State GetOrigin();
    std::vector<std::vector<double>> GetBoundary();
    void Play();
    int GetSwarmSize();
};

#endif  // INCLUDE_SWARM_ROBOTS_ARENA_HPP_        //  NOLINT

/**
 * @file arena.hpp
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
   public:  //  NOLINT
    /**
     * @brief Inilializes all agents in arena
     * @return bool: True if successful
     */
    bool InitializeAgents();

    // Method Not needed for current flow
    // State GetOrigin();
    // std::vector<std::vector<double>> GetBoundary();
    // void Play();

    /**
     * @brief Returns total active agents in swarm
     * @return int: swarm size
     */
    int GetSwarmSize();

   private:                       //  NOLINT
    const int total_agents = 20;  ///< Number of agents in arena
    vector<Agent> agents;         ///< Vector of all agent objects in arena
    // vector<double, double> boundary;
    // State origin;
};

#endif  // INCLUDE_SWARM_ROBOTS_ARENA_HPP_        //  NOLINT

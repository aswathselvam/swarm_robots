/**
 * @file arena.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The code file for arena attributes and methods
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */
#include "swarm_robots/arena.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "swarm_robots/agent.hpp"

bool Arena::InitializeAgents() {
    /*
    for (int i = 0; i < total_agents; i++) {
        ros::NodeHandle nh;
        agents.Initiaize(id, nh);
        agents.push_back(Agent());
    }
    */
    return true;
}

int Arena::GetSwarmSize() {
    return total_agents;
}

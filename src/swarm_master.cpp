/**
 * @file swarm_master.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief File for robot main execution methods
 * @version 0.2
 * @date 2021-12-11
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#include <string>
#include <vector>

#include "swarm_robots/agent_node.hpp"

using std::cout;
using std::string;
int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_master");

    vector<AgentNode*> agent_nodes;
    int n = 2;
    for (int i = 0; i < n; i++) {
        AgentNode* agent_node = new AgentNode(std::to_string(i));
        agent_nodes.push_back(agent_node);
    }

    while (ros::ok()) {
        for (int i = 0; i < n; i++) {
            agent_nodes[i]->Loop();
        }
    }

    return 0;
}

/**
 * @file agent.h
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for coordinate system used in the project
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#ifndef INCLUDE_SWARM_ROBOTS_AGENT_NODE_H_
#define INCLUDE_SWARM_ROBOTS_AGENT_NODE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include "state.h"    //  NOLINT
#include "swarm_robots/agent.h"

using std::vector;
using std::string;

class AgentNode : public Agent {

  public:
    AgentNode(std::string id);
    void PlanPath();
    void PerformInverseKinematics();
    void PerformForwardKinematics();
    void Loop();

  private :  // NOLINT
    int krate_;
    ros::NodeHandle* nh_;
    ros::Publisher vel_pub_;
    geometry_msgs::Twist twist_msg_;
    std::string agent_id;
};

#endif  // INCLUDE_SWARM_ROBOTS_AGENT_NODE_H_

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

#ifndef INCLUDE_SWARM_ROBOTS_AGENT_NODE_HPP_
#define INCLUDE_SWARM_ROBOTS_AGENT_NODE_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <vector>

#include "agent.hpp"
#include "state.hpp"  //  NOLINT

using std::string;
using std::vector;

class AgentNode : public Agent {
   public:  //  NOLINT
    explicit AgentNode(std::string id);
    ~AgentNode();
    void PlanPath();
    void PosCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void PerformInverseKinematics();
    void PerformForwardKinematics();
    void Loop();

   private:  // NOLINT
    int krate_;
    ros::NodeHandle* nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber pos_sub_;
    geometry_msgs::Twist twist_msg_;
    std::string agent_id;
};

#endif  // INCLUDE_SWARM_ROBOTS_AGENT_NODE_HPP_

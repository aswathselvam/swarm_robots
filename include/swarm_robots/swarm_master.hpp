/**
 * @file swarm_master.hpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for main code execution
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#ifndef INCLUDE_SWARM_ROBOTS_SWARM_MASTER_HPP_
#define INCLUDE_SWARM_ROBOTS_SWARM_MASTER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>

#include "agent.hpp"
using std::string;

class SwarmMaster {
   public:               //  NOLINT
    string nodename_;    ///< execution node nodename
    ros::NodeHandle n_;  ///< ros ros handle
    Agent agents;        ///< Agent class object

    /**
     * @brief SwarmMaster default constructor
     */
    SwarmMaster();
};
#endif  // INCLUDE_SWARM_ROBOTS_SWARM_MASTER_HPP_

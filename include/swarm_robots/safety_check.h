/**
 * @file safety_check.h
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for agent robot safety check
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */


#ifndef INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_        //  NOLINT
#define INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <utility>
#include <vector>
#include <string>
#include <agent_node.h>

using std::string;

class SafetyCheck {
    private:        //  NOLINT
        double error_tolerance;
        std::vector<std::pair<double, double>> distance;
        double max_range_;
        ros::Subscriber laser_sub_;
    public:        //  NOLINT
        bool IsSafe(AgentNode);
};
#endif  // INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_        //  NOLINT

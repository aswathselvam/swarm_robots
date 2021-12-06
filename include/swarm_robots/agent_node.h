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

using std::vector;

class Agent {
  private :  // NOLINT
    std::string robot_id;
    ros::NodeHandle nh;
    ros::Publisher state_pub_;
    ros::Publisher vel_pub_;
    ros::Subscriber obs_sub_;
    geometry_msgs::msg::Twist twist_;
    State state_;
    std::vector<double, double> destination_;
  public :  // NOLINT
    void Initialize();
    void PathPlanner();
    void InverseKinematics();
    void ForwardKinematics();
    void Stop();
};

#endif  // INCLUDE_SWARM_ROBOTS_AGENT_NODE_H_

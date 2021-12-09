/**
 * @file agent.h
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for agent control
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */
#ifndef INCLUDE_SWARM_ROBOTS_AGENT_H_
#define INCLUDE_SWARM_ROBOTS_AGENT_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <utility>
#include "path_planner.h"
#include "state.h"
#include "forward_kinematics.h"

using std::string;

class Agent : public FK {
    private:        //  NOLINT
        string robot_id_;
        ros::NodeHandle n_;
        ros::Publisher state_pub_;
        ros::Publisher vel_pub_;
        ros::Subscriber obs_sub_;
        geometry_msgs::Twist twist_;
        State state_;
        std::pair<double, double> destination_;

    public:        //  NOLINT
        void Initialize(ros::NodeHandle, string);
        void PathPlanner();
        void InverseKinematics();
        void ForwardKinematics();
        void Stop();
};
#endif  // INCLUDE_SWARM_ROBOTS_AGENT_H_

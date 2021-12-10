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
#include "swarm_robots/path_planner.h"
#include "swarm_robots/state.h"
#include "swarm_robots/forward_kinematics.h"
#include "swarm_robots/inverse_kinematics.h"
#include "swarm_robots/safety_check.h"

using std::string;

class Agent{

    public:        //  NOLINT
        Agent(string agent_id);
        void PlanPath();
        void PerformInverseKinematics();
        void PerformForwardKinematics();
        void Stop();

    protected:        //  NOLINT
        State position_;
        State velocity_;
        double heading_angle_;
        State goal_pos_;
        PathPlanner* path_planner_;
        ForwardKinematics* forward_kinematics_;
        InverseKinematics* inverse_kinematics_; 
    
    private:
        string agent_id_;
};
#endif  // INCLUDE_SWARM_ROBOTS_AGENT_H_

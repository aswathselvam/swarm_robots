/**
 * @file agent.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The code file for agent control
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cmath>
#include "swarm_robots/agent.h"
#include "swarm_robots/arena.h"
#include "swarm_robots/path_planner.h"
#include "swarm_robots/state.h"
#include "swarm_robots/forward_kinematics.h"
#include "swarm_robots/safety_check.h"


using std::string;

void Agent::Agent(string agent_id) {    
    this->agent_id = agent_id;
}

void Agent::PlanPath(){
    pathplanner_->PathPlanner();
}

void Agent::PerformInverseKinematics(){
      if(!pathplanner_.waypoints.size()>0)
        PlanPath();

    State intermediate_goal = pathplanner.waypoints.begin();
    this->velocity_ = inverse_kinemaics_->PerformIK(intermediate_goal, this->position_);
}

void Agent::PerformForwardKinematics(){
  
}

void Agent::Stop(){
    velocity_.x=0;
    velocity_.y=0;
    velocity_.yaw=0;
}

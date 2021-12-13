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

#include "swarm_robots/agent.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cmath>

#include "swarm_robots/arena.hpp"
#include "swarm_robots/forward_kinematics.hpp"
#include "swarm_robots/inverse_kinematics.hpp"
#include "swarm_robots/path_planner.hpp"
#include "swarm_robots/safety_check.hpp"
#include "swarm_robots/state.hpp"

using std::string;

Agent::Agent(string agent_id) {
    this->agent_id_ = agent_id;
    this->position_.x_ = 0;
    this->position_.y_ = 0;
    this->position_.yaw_ = 0;
    this->velocity_.x_ = 0;
    this->velocity_.y_ = 0;
    this->velocity_.yaw_ = 0;
    this->heading_angle_ = 180;
}

void Agent::PlanPath() {
    // path_planner_->Plan(this->position_, this->goal_pos_);
}

void Agent::PerformInverseKinematics() {
    /*
      if(!path_planner_->waypoints_.size()>0)
        PlanPath();
    
    State intermediate_goal = *( path_planner_->waypoints_.begin() );
    */

    //Provide a Sample goal state: 
    //State intermediate_goal(2, 4);
    
    State intermediate_goal = path_planner_->GetContactPoint(std::stoi(agent_id_));
    this->velocity_ =
        inverse_kinematics_->PerformIK( this->position_, intermediate_goal);

    /*
    double dist = sqrt(pow(intermediate_goal.x_ - this->position_.x_,2) + pow(intermediate_goal.y_ - this->position_.y_,2));
    double THRESHOLD = 1;
    if(dist<THRESHOLD){
        path_planner_->waypoints_.pop_back();
    }
    */
}

void Agent::PerformForwardKinematics() {
    this->velocity_ = forward_kinematics_->PerformFK(this->velocity_);
}

void Agent::Stop() {
    velocity_.x_ = 0;
    velocity_.y_ = 0;
    velocity_.yaw_ = 0;
}

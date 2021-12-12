/**
 * @file agent_node.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for coordinate system used in the project
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <string>

#include "../include/swarm_robots/agent_node.hpp"
#include "../include/swarm_robots/agent.hpp"
#include "../include/swarm_robots/path_planner.hpp"

using std::string;

AgentNode::AgentNode(std::string ns) : Agent::Agent(ns) {
    this->agent_id = ns;
    int argc;
    char** argv;
    ros::init(argc, argv, "id");
    this->nh_ = new ros::NodeHandle();

    this->path_planner_ = new PathPlanner(ns, nh_);
    this->forward_kinematics_ = new ForwardKinematics();
    this->inverse_kinematics_ = new InverseKinematics(ns, nh_);

    // agent1/cmd_vel
    this->vel_pub_ = this->nh_->advertise<geometry_msgs::Twist>("agent" + ns + "/cmd_vel", this->krate_, this);
    this->krate_ = 20;
}

void AgentNode::Loop() {
    ros::Rate loop_rate(this->krate_);
    while (ros::ok()) {
        AgentNode::PlanPath();
        AgentNode::PerformInverseKinematics();
        AgentNode::PerformForwardKinematics();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void AgentNode::PlanPath() {
    Agent::PlanPath();
}

void AgentNode::PerformInverseKinematics() {
    Agent::PerformInverseKinematics();
}

void AgentNode::PerformForwardKinematics() {
    Agent::PerformForwardKinematics();
    twist_msg_.linear.x = velocity_.x_;
    twist_msg_.linear.y = 0;
    twist_msg_.linear.z = 0;
    twist_msg_.angular.x = 0;
    twist_msg_.angular.y = 0;
    twist_msg_.angular.z = velocity_.yaw_;
    vel_pub_.publish(twist_msg_);
}

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

#include "swarm_robots/agent.h"

AgentNode::AgentNode(std::string id): agent_id(id), Agent(id) {
  ros::init("agent_" + agent_id + "_node");
  this->nh_ = new ros::NodeHandle();
  this->vel_pub_ ros::Publisher(); = obstacle_sub_ = twist_ =
  this->krate_ = 20;
}

AgentNode::Loop(){
  ros::Rate loop_rate(this->krate_);
  while(ros::ok()){
    AgentNode::PlanPath();
    AgentNode::InverseKinematics();
    AgentNode::ForwardKinematics();
    ros::spinOnce();
    loop_rate.sleep();
  }
}


AgentNode::PlanPath(){
      Agent::PlanPath();
}

AgentNode::InverseKinematics(){
    Agent::InverseKinematics();
}

AgentNode::ForwardKinematics(){
  Agent::ForwardKinematics();
  twist_msg_.linear.x = velocity_.x;
  twist_msg_.linear.y = 0;
  twist_msg_.linear.z = 0;
  twist_msg_.angular.x = 0;
  twist_msg_.angular.y = 0;
  twist_msg_.angular.z = velocity_.yaw;
  vel_pub_.publish(twist_msg_);
}

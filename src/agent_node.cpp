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

#include "../include/swarm_robots/agent_node.hpp"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <string>

#include "../include/swarm_robots/agent.hpp"
#include "../include/swarm_robots/path_planner.hpp"

using std::string;

AgentNode::AgentNode(std::string ns) : Agent::Agent(ns) {
    this->agent_id = ns;
    this->nh_ = new ros::NodeHandle();
    this->path_planner_ = new PathPlanner(ns, nh_);
    this->forward_kinematics_ = new ForwardKinematics();
    this->inverse_kinematics_ = new InverseKinematics(ns, nh_);

    // agent1/cmd_vel
    this->vel_pub_ = this->nh_->advertise<geometry_msgs::Twist>
      ("/jackal" + ns + "/jackal_velocity_controller/cmd_vel", this->krate_, this);
    this->pos_sub_ = this->nh_->subscribe("/jackal" + ns + "/base_pose_ground_truth",
                                          1, &AgentNode::PosCallback, this);

    this->krate_ = 20;
    Loop();
}

void AgentNode::Loop() {
    ros::Rate loop_rate(this->krate_);
    // while(ros::ok()){
    AgentNode::PlanPath();
    AgentNode::PerformInverseKinematics();
    AgentNode::PerformForwardKinematics();
    ros::spinOnce();
    // loop_rate.sleep();
    //}
}

void AgentNode::PosCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    this->position_.x_ = msg->pose.pose.position.x;
    this->position_.y_ = msg->pose.pose.position.y;

    tf::Quaternion quat;
    quat[0] = msg->pose.pose.orientation.x;
    quat[1] = msg->pose.pose.orientation.y;
    quat[2] = msg->pose.pose.orientation.z;
    quat[3] = msg->pose.pose.orientation.w;

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    this->position_.yaw_ = yaw;

    // ROS_INFO("Seq: [%d]", msg->header.seq);
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

AgentNode::~AgentNode() {
    delete nh_;
    std::cout << "Deteletd NH";
}

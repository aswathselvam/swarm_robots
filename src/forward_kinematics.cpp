/**
 * @file forward_kinematics.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief File for robot motion methods
 * @version 0.2
 * @date 2021-12-11
 * @copyright BSD3 Copyright (c) 2021
 *
 */
#include "../include/swarm_robots/forward_kinematics.hpp"

#include <algorithm>
#include <iostream>

using std::max;
using std::min;

ForwardKinematics::ForwardKinematics() {
    kDriveVelocityLimit = 10;
    kSteerVelocityLimit = 5;
}

State ForwardKinematics::PerformFK(State velocity) {
    if (velocity_.x_ > 0) {
        velocity_.x_ = min(kDriveVelocityLimit, velocity.x_);
    } else {
        velocity_.x_ = max(-kDriveVelocityLimit, velocity.x_);
    }

    if (velocity_.yaw_ > 0) {
        velocity_.yaw_ = min(kSteerVelocityLimit, velocity.yaw_);
    } else {
        velocity_.yaw_ = max(-kSteerVelocityLimit, velocity.yaw_);
    }

    // std::cout<<"Final velocity yaw: "<<velocity_.x_;

    return velocity_;
}

bool ForwardKinematics::MoveForward(geometry_msgs::Twist *robot_vel) {
    robot_vel->linear.x = 0.3;
    robot_vel->linear.y = 0.0;
    robot_vel->linear.z = 0.0;
    robot_vel->angular.x = 0.0;
    robot_vel->angular.y = 0.0;
    robot_vel->angular.z = 0.0;
    ROS_INFO_STREAM("Move forward Velocity:" << robot_vel->linear.x);
    return true;
}

bool ForwardKinematics::Stop(geometry_msgs::Twist *robot_vel) {
    robot_vel->linear.x = 0.0;
    robot_vel->linear.y = 0.0;
    robot_vel->linear.z = 0.0;
    robot_vel->angular.x = 0.0;
    robot_vel->angular.y = 0.0;
    robot_vel->angular.z = 0.0;
    return true;
}

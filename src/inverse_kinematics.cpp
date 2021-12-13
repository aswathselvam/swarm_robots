/**
 * @file inverse_kinematics.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief File for robot IK methods
 * @version 0.2
 * @date 2021-12-11
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#include "swarm_robots/inverse_kinematics.hpp"

#include <iostream>
#include <cmath>
#include <string>

#include "swarm_robots/state.hpp"

InverseKinematics::InverseKinematics(string ns, ros::NodeHandlePtr nh) {
    // safety_check_ = new SafetyCheck(ns, nh); // CPP CHECK Error
    SafetyCheck safety_check(ns, nh);
    safety_check_ = &safety_check;
}

State InverseKinematics::PerformIK(State start, State goal) {
    this->current_location_ = start;
    this->goal_location_ = goal;
    State vel =  PerformModelIK();
    State rep_vel = CheckSafety();
    return vel+rep_vel;
}

State InverseKinematics::PerformModelIK() {
    State velocity;
    double dist = sqrt(pow(current_location_.x_ - goal_location_.x_, 2) +
                       pow(current_location_.y_ - goal_location_.y_, 2));
    double target_yaw = atan2(goal_location_.y_ - current_location_.y_,
                              goal_location_.x_ - current_location_.x_);
    double heading_angle_ = current_location_.yaw_;
    velocity.yaw_ = target_yaw-heading_angle_;
    
    //Debug Target and Heading angles:
    //std::cout<<"target yaw: "<<target_yaw<<std::endl;
    //std::cout<<"heading_angle : "<<heading_angle_<<std::endl;
    
    float Kp = 1;
    // Policy 1:
    // velocity_.x=Kp*dist;

    // Policy 2:
    velocity.x_ = Kp * abs(dist) / dist;

    return velocity;
}

State InverseKinematics::CheckSafety() {
    State safe_state(0, 0);
    safe_state = safety_check_->GetSafeDirection();
    return safe_state;
}

/**
 * @file forward_kinematics.h
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for agent FK
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#ifndef INCLUDE_SWARM_ROBOTS_FORWARD_KINEMATICS_HPP_
#define INCLUDE_SWARM_ROBOTS_FORWARD_KINEMATICS_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>

#include "path_planner.hpp"
#include "state.hpp"  //  NOLINT

using std::string;

class ForwardKinematics {
   public:  //  NOLINT
    ForwardKinematics();
    State PerformFK(State velocity);

    bool MoveForward(geometry_msgs::Twist *robot_vel);
    bool Stop(geometry_msgs::Twist *robot_vel);

   private:  //  NOLINT
    State velocity_;
    double kDriveVelocityLimit;
    double kSteerVelocityLimit;
};
#endif  // INCLUDE_SWARM_ROBOTS_FORWARD_KINEMATICS_HPP_

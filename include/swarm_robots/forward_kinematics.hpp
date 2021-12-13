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
    /**
     * @brief ForwardKinematics default constructor
     */
    ForwardKinematics();

    /**
     * @brief Perform forward kinematics to reach next state
     * @param State velocity : current agent x,y,z velocities
     * @return State: Output agent x,y,z velocities
     */
    State PerformFK(State velocity);

    /**
     * @brief Set velocity to move agent forward
     * @param geometry_msgs::Twist robot_vel : reference to velocity variable
     * @return bool: True is velocities set successfully
     */
    bool MoveForward(geometry_msgs::Twist *robot_vel);

    /**
     * @brief Set velocity to stop agent motion
     * @param geometry_msgs::Twist robot_vel : reference to velocity variable
     * @return bool: True is velocities set successfully
     */
    bool Stop(geometry_msgs::Twist *robot_vel);

   private:                      //  NOLINT
    State velocity_;             ///< Variable to store x,y,z velocity coordinares
    double kDriveVelocityLimit;  ///< Max drive velocity
    double kSteerVelocityLimit;  ///< Max steer velocity
};
#endif  // INCLUDE_SWARM_ROBOTS_FORWARD_KINEMATICS_HPP_

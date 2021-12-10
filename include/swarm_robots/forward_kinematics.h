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


#ifndef INCLUDE_SWARM_ROBOTS_FORWARD_KINEMATICS_H_        //  NOLINT
#define INCLUDE_SWARM_ROBOTS_FORWARD_KINEMATICS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include "path_planner.h"
#include "state.h"        //  NOLINT

using std::string;

class ForwardKinematics {
    public:        //  NOLINT
        ForwardKinematics();
        State PerformFK(State velocity);

    private:        //  NOLINT
        State velocity_;
        double kDriveVelocityLimit;
        double kSteerVelocityLimit;

};
#endif  // INCLUDE_SWARM_ROBOTS_FORWARD_KINEMATICS_H_        //  NOLINT

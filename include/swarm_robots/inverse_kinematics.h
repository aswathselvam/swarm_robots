/**
 * @file inverse_kinematics.h
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for agent FK
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */


#ifndef INCLUDE_SWARM_ROBOTS_IK_H_        //  NOLINT
#define INCLUDE_SWARM_ROBOTS_IK_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include "state.h"        //  NOLINT
#include "safety_check.h"        //  NOLINT

using std::string;

class IK {
    private:        //  NOLINT
        State reference_point;
        double drive_velocity;
        double steering_val;
    public:        //  NOLINT
        void SafetyCheck();
        void PerformModelIK();
        bool PerformIK();
};
#endif  // INCLUDE_SWARM_ROBOTS_IK_H_        //  NOLINT

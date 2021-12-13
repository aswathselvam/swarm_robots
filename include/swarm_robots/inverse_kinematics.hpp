/**
 * @file inverse_kinematics.h
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for agent IK
 * @version 0.2
 * @date 2021-12-12
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#ifndef INCLUDE_SWARM_ROBOTS_INVERSE_KINEMATICS_HPP_  //  NOLINT
#define INCLUDE_SWARM_ROBOTS_INVERSE_KINEMATICS_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>

#include "safety_check.hpp"  //  NOLINT
#include "state.hpp"         //  NOLINT

using std::string;

class InverseKinematics {
   public:  //  NOLINT
    /**
     * @brief InverseKinematics parameterized constructor,
     * initialize node handle with given namespace
     * @param string ns : namespace of agent node
     * @param string nh : reference to node handle
     */
    InverseKinematics(string ns, ros::NodeHandle* nh);

    /**
     * @brief Invoke PerformModelIK() and CheckSafety()
     * @param State start : current coordintes
     * @param State goal : end goal coordinates
     * @return State: Output agent x,y,z velocities
     */
    State PerformIK(State start, State goal);

    /**
     * @brief Compute goal velocity to reach goal state
     * @return State: Output agent x,y,z velocities
     */
    State PerformModelIK();

    /**
     * @brief Check if agent is safe from collissions
     * @return State: Output x,y,x coordinates
     */
    State CheckSafety();

   private:                      //  NOLINT
    State goal_location_;        ///< Goal x,y,z coordinares
    State current_location_;     ///< Current x,y,z coordinares
    State velocity_;             ///< Variable to store x,y,z velocities
    SafetyCheck* safety_check_;  ///< reference to Safety_check class
};
#endif  // INCLUDE_SWARM_ROBOTS_INVERSE_KINEMATICS_HPP_        //  NOLINT

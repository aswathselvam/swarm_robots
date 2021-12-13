/**
 * @file agent.hpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for agent control
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */
#ifndef INCLUDE_SWARM_ROBOTS_AGENT_HPP_
#define INCLUDE_SWARM_ROBOTS_AGENT_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <utility>

#include "forward_kinematics.hpp"
#include "inverse_kinematics.hpp"
#include "path_planner.hpp"
#include "safety_check.hpp"
#include "state.hpp"

using std::string;

class Agent {
   public:  //  NOLINT
    /**
     * @brief Agent parameterized constructor,
     * initialized agent with given id
     * @param id : id of the agent - assigned to initialized
     * Agent class object
     */
    explicit Agent(string agent_id);

    /**
     * @brief Invoke agent path planeer
     */
    void PlanPath();

    /**
     * @brief Invokes IK method from inverse kinematics class
     */
    void PerformInverseKinematics();

    /**
     * @brief Invokes FK method from forward kinematics class
     */
    void PerformForwardKinematics();

    /**
     * @brief Sets all velocities to zero
     */
    void Stop();

   protected:                    //  NOLINT
    State position_;           ///< Variable to store coordinates of agent
    State velocity_;           ///< Variable to store x,y,z velocities of agent
    double heading_angle_;     ///< Variable to calculate agngle for IK
    State goal_pos_;           ///< Variable to store x,y,z of destination
    PathPlanner* path_planner_;  ///< PathPlanner object for agent node

    ForwardKinematics* forward_kinematics_;  ///< ForwardKinematics object for agent node   // NOLINT

    ///< InverseKinematics object for agent node
    InverseKinematics* inverse_kinematics_;

   private:            //  NOLINT
    string agent_id_;  ///< ID of agent to refer it's namespace
};
#endif  // INCLUDE_SWARM_ROBOTS_AGENT_HPP_

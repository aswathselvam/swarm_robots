/**
 * @file safety_check.h
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for agent robot Path Planner
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#ifndef INCLUDE_SWARM_ROBOTS_PATH_PLANNER_HPP_
#define INCLUDE_SWARM_ROBOTS_PATH_PLANNER_HPP_

#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <vector>

#include "state.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using std::string;
using std::vector;

class PathPlanner {
   public:  //  NOLINT
    State start_;        ///< start coordinates
    State goal_;        ///< end goal coordinates
    vector<State> waypoints_;        ///< vector of intermediate waypounts
    bool success_;        ///< sucess/fail flag

    /**
     * @brief PathPlanner parameterized constructor,
     * initializes agent namespace and nodehandle
     * @param string ns : agent namespace
     */
    PathPlanner(std::string ns, ros::NodeHandle* nh);

    /**
     * @brief Creates OMPL empty map for path planning
     */
    void CreateEmptyMap();

    /**
     * @brief Returns if state is valid
     * @param const ob::State* state : State coordinates
     * @return bool: True if state is valid
     */
    bool IsStateValid(const ob::State* state);

    /**
     * @brief Main path plan method
     * @param State state : agent current coordinates
     * @param State goal : agent goal coordinates
     * @return bool: True if successful
     */
    bool Plan(State start, State goal);

   private:  //  NOLINT
    //  ROS publishers
    string ns;        ///< Agent namespace
    ros::NodeHandle* nh_;        ///< ROS node handle
    string fixed_frame;        ///< Default value : "map"
    ros::Publisher vis_pub_;        ///< ros velocity publisher
    octomap::OcTree* oct_tree_;        ///< oct_tree_ to store map
    ob::StateSpacePtr space;        ///< State space pointer
    ob::SpaceInformationPtr si;        ///< space information pointer
};
#endif  // INCLUDE_SWARM_ROBOTS_PATH_PLANNER_HPP_

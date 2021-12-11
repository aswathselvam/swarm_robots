/**
 * @file path_planner.h
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for agent path plan methods
 * @version 0.2
 * @date 2021-12-11
 * @copyright BSD3 Copyright (c) 2021
 *
 */
#ifndef INCLUDE_SWARM_ROBOTS_PATH_PLANNER_H_
#define INCLUDE_SWARM_ROBOTS_PATH_PLANNER_H_

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

#include <iostream>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using std::string;
using std::vector;

class PathPlanner {
   public:  //   NOLINT
    State start_;
    State goal_;
    vector<State> waypoints_;
    bool success_;
    PathPlanner(std::string ns, ros::NodeHandle* nh);
    void CreateEmptyMap();
    bool IsStateValid(const ob::State* state);
    bool Plan(State start, State goal);

    /**
     * @brief Method calculate x,y,yaw pose for agent-object contact point
     * @param agent_id id of agent node for which pose is to be calculated
     * @param nh ROS node handler
     * @return State class object with set values for pose : x,y,yaw
     */
    State GetContactPoint(int agent_id, ros::NodeHandle* nh);

   private:  //   NOLINT
    // ROS publishers
    string ns;
    ros::NodeHandle* nh_;
    string fixed_frame;
    ros::Publisher vis_pub_;
    octomap::OcTree* oct_tree_;
};
#endif  // INCLUDE_SWARM_ROBOTS_PATH_PLANNER_H_

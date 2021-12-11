#ifndef INCLUDE_SWARM_ROBOTS_PATH_PLANNER_H_
#define INCLUDE_SWARM_ROBOTS_PATH_PLANNER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <vector>

#include "swarm_robots/state.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using std::string;
using std::vector;


class PathPlanner {

public:
    State start_;
    State goal_;
    vector<State> waypoints_;
    bool success_;
    PathPlanner(std::string ns, ros::NodeHandle* nh);
    void CreateEmptyMap();
    bool IsStateValid(const ob::State *state);
    bool Plan(State start, State goal);

private:
    //ROS publishers
    string ns;
    ros::NodeHandle* nh_;
    string fixed_frame;
    ros::Publisher vis_pub_;
    octomap::OcTree *oct_tree_;
    ob::StateSpacePtr space;
    ob::SpaceInformationPtr si;


};
#endif  // INCLUDE_SWARM_ROBOTS_PATH_PLANNER_HPP_

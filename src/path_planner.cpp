/**
 * @file forward_kinematics.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief File for robot path planning methodss
 * @version 0.2
 * @date 2021-12-11
 * @copyright BSD3 Copyright (c) 2021
 *
 */
#include "swarm_robots/path_planner.hpp"

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

#include <algorithm>

#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "swarm_robots/state.hpp"
#include "tf/transform_broadcaster.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using std::string;

// #define DEBUG

#ifdef DEBUG
#define DEBUG_MSG(str)                 \
    do {                               \
        std::cout << str << std::endl; \
    } while (false)
#else
#define DEBUG_MSG(str) \
    do {               \
    } while (false)
#endif

PathPlanner::PathPlanner(std::string ns, ros::NodeHandle *nh) {
    this->ns = "_jackal_" + ns;
    this->fixed_frame = "map";
    this->nh_ = nh;
    // ros::Subscriber octree_sub =
    // n.subscribe("/octomap_binary", 1, octomapCallback);
    this->vis_pub_ =
        nh_->advertise<visualization_msgs::Marker>("visualization_marker" + this->ns, 0);  // NOLINT
    DEBUG_MSG("OMPL version: " << OMPL_VERSION << std::endl);

    // space = ob::StateSpacePtr(new ob::SE2StateSpace());
}

bool PathPlanner::IsStateValid(const ob::State *state) {
    return true;

    const ob::RealVectorStateSpace::StateType *pos =
        state->as<ob::RealVectorStateSpace::StateType>();

    // check validity of state defined by pos
    // fcl::Vector3<double>
    // translation(pos->values[0],pos->values[1],pos->values[2]);

    octomap::point3d query(pos->values[0], pos->values[1], 0);
    octomap::OcTreeNode *result = oct_tree_->search(query);

    if ((result != NULL) && (result->getValue() >= 0.5f)) {
        return true;
    } else {
        return false;
    }
}

bool PathPlanner::Plan(State start, State goal) {
    /*
    // construct the state space we are planning in

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start_state(space);
    start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_.x_;
    start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_.y_;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal_state(space);
    goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_.x_;
    goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_.y_;

// create a problem instance
ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

// set the start and goal states
og::PathGeometric *pth;
pdef->setStartAndGoalStates(start_state, goal_state);

// pdef->setOptimizationObjective(getPathLengthObjective(si));

// create a planner for the defined space
ob::PlannerPtr planner(new og::RRTConnect(si));

// set the problem we are trying to solve for the planner
planner->setProblemDefinition(pdef);

// perform setup steps for the planner
planner->setup();

// print the settings for this space
DEBUG_MSG(si->printSettings(std::cout));
// print the problem settings
DEBUG_MSG(pdef->print(std::cout));

// attempt to solve the problem within one second of planning time
ob::PlannerStatus solved = planner->solve(0.5);

if (solved) {
    this->success_ = true;
    // get the goal representation from the problem definition (not the same as the goal state)
    // and inquire about the found path
    std::cout << "Found solution:" << std::endl;
    ob::PathPtr path = pdef->getSolutionPath();
    pth = pdef->getSolutionPath()->as<og::PathGeometric>();
    pth->printAsMatrix(std::cout);

    // Publish path as markers
    visualization_msgs::Marker path_marker;
    path_marker.action = visualization_msgs::Marker::DELETEALL;
    vis_pub_.publish(path_marker);

    for (std::size_t idx = 0; idx < pth->getStateCount(); idx++) {
        // cast the abstract state type to the type we expect
        const ob::RealVectorStateSpace::StateType *path_node = pth->getState(idx)->as<ob::RealVectorStateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        // const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        // const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        State node(path_node->values[0], path_node->values[1]);
        this->waypoints_.push_back(node);

        path_marker.header.frame_id = fixed_frame;
        path_marker.header.stamp = ros::Time();
        path_marker.ns = "path" + this->ns;
        path_marker.id = idx;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point tmp_p;

        tmp_p.x = path_node->values[0];
        tmp_p.y = path_node->values[1];
        path_marker.points.push_back(tmp_p);
        path_marker.scale.x = 0.1;
        path_marker.scale.y = 0.1;
        path_marker.scale.z = 0.1;
        path_marker.pose.orientation.x = 0;
        path_marker.pose.orientation.y = 0;
        path_marker.pose.orientation.z = 0;
        path_marker.pose.orientation.w = 1;
        path_marker.color.a = 1.0;
        path_marker.color.r = 1.0;
        path_marker.color.g = 1.0;
        path_marker.color.b = 0.0;
        vis_pub_.publish(path_marker);
        ros::Duration(0.001).sleep();
    }
    std::reverse(waypoints_.begin(), waypoints_.end());

} else {
    this->success_ = false;
    DEBUG_MSG("No solution found" << std::endl);
}

return success_;
    */
   return true;
}

void PathPlanner::CreateEmptyMap() {
    /*
this->oct_tree_ = new octomap::OcTree(0.01);
    for (int x =0 ; x<10; x++){
            for (int y =0 ; y<10; y++){
                    octomap::point3d endpoint((float)x * 0.01f* 1, (float)y * 0.01f * 1, 0.0f);
                    oct_tree_->updateNode(endpoint, false);
            }
    }

    */
}

State PathPlanner::GetContactPoint(int agent_id) {
    ros::ServiceClient client =
        nh_->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState getModelState;
    geometry_msgs::Point pp;
    // geometry_msgs::Quaternion qq;
    std::string modelName = (std::string) "unit_box_custom";
    std::string relativeEntityName = (std::string) "world";
    getModelState.request.model_name = modelName;
    getModelState.request.relative_entity_name = relativeEntityName;
    client.call(getModelState);

    pp = getModelState.response.pose.position;
    // qq = getModelState.response.pose.orientation;
    std::string x_ = (std::to_string(pp.x));
    ROS_INFO("Test Box x coordinate [%s]", x_.c_str());
    std::cout << x_;
    State pose_agent(0, 0, 0);
    if (agent_id < 11) {
        ROS_INFO("Contact point : longer side");
        int left_right = (agent_id > 5 ? 1 : -1);
        pose_agent.x_ = pp.x + (left_right * (3.13 + ((agent_id - 1) * 0.7)));
        pose_agent.y_ = pp.y + 2.24;
        // pose_agent.yaw_ =
    } else {
        pose_agent.x_ = 0;
        pose_agent.y_ = 0;
    }
    return pose_agent;
}

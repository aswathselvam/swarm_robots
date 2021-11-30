#ifndef INCLUDE_SWARM_ROBOTS_AGENT_H_
#define INCLUDE_SWARM_ROBOTS_AGENT_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "path_planner.h"
#include "state.h"

using std::string;

class Agent {
    string robot_id_;
    ros::NodeHandle n_;
    ros::Publisher state_pub_;
    ros::Publisher vel_pub_;
    ros::Subscriber obs_sub_;
    geometry_msgs::Twist twist_;
    State state_;
    std::pair<double, double> destination_;


void Initialize();
void PathPlanner();
void InverseKinematics();
void ForwardKinematics();
void Stop();
};
#endif  // INCLUDE_SWARM_ROBOTS_AGENT_H_

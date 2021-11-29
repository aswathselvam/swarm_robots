#ifndef INCLUDE_SWARM_ROBOTS_SWARM_MASTER_H_
#define INCLUDE_SWARM_ROBOTS_SWARM_MASTER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include "swarm_robots/agent.h"
using std::string;

class SwarmMaster {
    string nodename_;
    ros::NodeHandle n_;
    Agent agents;

};
#endif  // INCLUDE_SWARM_ROBOTS_SWARM_MASTER_H_

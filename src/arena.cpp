#include "swarm_robots/agent.h"
#include "swarm_robots/arena.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

bool Arena::InitializeAgents() {
    for(int i=0; i<20;i++) {
        ros::NodeHandle nh;
        agents.Initiaize(); //id, nh);
        agents.push_back(Agent());
    }
    return true;
}
#ifndef INCLUDE_SWARM_ROBOTS_ARENA_HPP_
#define INCLUDE_SWARM_ROBOTS_ARENA_HPP_

#include <swarm_robots/agent.h>
#include <swarm_robots/state.h>
#include <std_msgs/String.h>
#include <vector>

using std::vector;
class Arena{
    private:

    vector<Agent> agents;
    vector<double, double> boundary;
    State origin;

    void InitializeAgents();
    State GetOrigin();
    vector GetBoundary();
};

#endif  // INCLUDE_SWARM_ROBOTS_ARENA_HPP_

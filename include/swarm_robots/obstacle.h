#ifndef INCLUDE_SWARM_ROBOTS_OBSTACLE_H_
#define INCLUDE_SWARM_ROBOTS_OBSTACLE_H_

#include <string>
using namespace string;

class Obstacle{
public:
    Obstacle(string ns);
    UpdateObstacleLocation();
    virtual double x_;
    virtual double y_;
    virtual double length_;
    virtual double width_;

};


#endif  // INCLUDE_SWARM_ROBOTS_OBSTACLE_H_

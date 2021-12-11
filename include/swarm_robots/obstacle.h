#ifndef INCLUDE_SWARM_ROBOTS_OBSTACLE_H_
#define INCLUDE_SWARM_ROBOTS_OBSTACLE_H_

#include <string>
using std::string;

class Obstacle{
public:
    Obstacle(string ns);
    void UpdateObstacleLocation();
    double x_;
    double y_;
    double length_;
    double width_;

};


#endif  // INCLUDE_SWARM_ROBOTS_OBSTACLE_H_

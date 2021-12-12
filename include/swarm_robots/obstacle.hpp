/**
 * @file swarm_master.hpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for arena obstacles
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#ifndef INCLUDE_SWARM_ROBOTS_OBSTACLE_HPP_
#define INCLUDE_SWARM_ROBOTS_OBSTACLE_HPP_

#include <string>
using std::string;

class Obstacle {
   public:  //  NOLINT
    explicit Obstacle(string ns);
    void UpdateObstacleLocation();
    double x_;
    double y_;
    double length_;
    double width_;
};

#endif  // INCLUDE_SWARM_ROBOTS_OBSTACLE_HPP_

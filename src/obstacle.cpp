/**
 * @file main.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief File for obstacle dependent methods
 * @version 0.2
 * @date 2021-12-11
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#include "swarm_robots/obstacle.hpp"

#include <string>

using std::string;

Obstacle::Obstacle(string ns) {
    if (!ns.empty()) {
        x_ = 0;
        y_ = 0;
        length_ = 1;
        width_ = 1;
    }
}

bool Obstacle::UpdateObstacleLocation(double x, double y) {
    x_ = x;
    y_ = y;
    return true;
}

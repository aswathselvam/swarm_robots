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
    /**
     * @brief AgentNode parameterized constructor,
     * initializes agent namespace
     * @param string ns : agent namespace
     */
    explicit Obstacle(string ns);

    /**
     * @brief Set values to Obstacle x,y, and dimensions
     */
    void UpdateObstacleLocation();

    double x_;       ///< Obstacle x coordinate
    double y_;       ///< Obstacle y coordinate
    double length_;  ///< Obstacle length
    double width_;   ///< Obstacle width
};

#endif  // INCLUDE_SWARM_ROBOTS_OBSTACLE_HPP_

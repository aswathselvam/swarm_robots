/**
 * @file state.hpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for state of agents
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#ifndef INCLUDE_SWARM_ROBOTS_STATE_HPP_
#define INCLUDE_SWARM_ROBOTS_STATE_HPP_

class State {
   public:  //  NOLINT
    /**
     * @brief State default constructor
     */
    State();

    /**
     * @brief State parameterized constructor,
     * initializes x,y state coordinates
     */
    State(double x, double y);

    /**
     * @brief State parameterized constructor,
     * initializes x,y,yaw state coordinates
     */
    State(double x, double y, double yaw);

    /**
     * @brief Add 2 States, + operator overloading,
     * @param state First state variable.
     * @return State output the new sum of 2 states.
     */
    State operator+(State const &state);

    double x_;    ///< Sate x coordinate
    double y_;    ///< Sate y coordinate
    double yaw_;  ///< Sate yaw value
};

#endif  // INCLUDE_SWARM_ROBOTS_STATE_HPP_

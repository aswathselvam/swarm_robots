/**
 * @file state.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief File for robot state methods
 * @version 0.2
 * @date 2021-12-11
 * @copyright BSD3 Copyright (c) 2021
 *
 */
#include "../include/swarm_robots/state.hpp"

State::State() {
    this->x_ = 0;
    this->y_ = 0;
    this->yaw_ = 0;
}

State::State(double x, double y) {
    this->x_ = x;
    this->y_ = y;
    this->yaw_ = 0;
}

State::State(double x, double y, double yaw) {
    this->x_ = x;
    this->y_ = y;
    this->yaw_ = yaw;
}
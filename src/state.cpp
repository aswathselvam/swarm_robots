#include "swarm_robots/state.h"


State::State(){
    this->x_=0;
    this->y_=0;
    this->yaw_=0;
}

State::State(double x, double y) {
    this->x_ = x;
    this->y_ = y;
    this->yaw_ = 0;
} 

State::State(double x, double y, double yaw){
    this->x_ = x;
    this->y_ = y;
    this->yaw_ = yaw;
}
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

State State::operator + (State const &state) {
        State result;
        result.x_ = x_ + state.x_;
        result.y_ = y_ + state.y_;
        result.yaw_ = yaw_ + state.yaw_;
        return result;
}
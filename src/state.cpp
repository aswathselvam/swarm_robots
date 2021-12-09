#include "state.h"


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
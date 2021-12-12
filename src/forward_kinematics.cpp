#include <algorithm> 
#include <iostream>
#include "swarm_robots/forward_kinematics.h"

using std::min;
using std::max;


ForwardKinematics::ForwardKinematics(){
    kDriveVelocityLimit = 10;
    kSteerVelocityLimit = 5;
}

State ForwardKinematics::PerformFK(State velocity){

    if(velocity_.x_ > 0){
        velocity_.x_ = min(kDriveVelocityLimit, velocity.x_);
    }else{
        velocity_.x_ = max(-kDriveVelocityLimit, velocity.x_);
    }

    if(velocity_.yaw_ > 0){
        velocity_.yaw_ = min(kSteerVelocityLimit, velocity.yaw_);
    }else{
        velocity_.yaw_ = max(-kSteerVelocityLimit, velocity.yaw_);
    }

    //std::cout<<"Final velocity yaw: "<<velocity_.x_;

    return velocity_;
}



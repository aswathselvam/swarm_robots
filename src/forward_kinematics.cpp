#include <algorithm> 
#include "swarm_robots/forward_kinematics.h"

using std::max;

ForwardKinematics::ForwardKinematics(){
    kDriveVelocityLimit = 10;
    kSteerVelocityLimit = 5;
}

State ForwardKinematics::PerformFK(State velocity){
    velocity_.x_ = max(kDriveVelocityLimit, velocity.x_);
    velocity_.yaw_ = max(kSteerVelocityLimit, velocity.yaw_);
    return velocity_;
}



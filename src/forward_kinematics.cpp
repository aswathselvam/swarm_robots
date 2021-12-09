#include "swarm_master/forward_kinematics.h"
#include <algorithm> 

State ForwardKinematics::PerformFK(State velocity){
    velocity.x = max(kDriveVelocityLimit, velocity_.x);
    velocity.yaw = max(kSteerVelocityLimit, velocity_.yaw);
    return velocity;
}


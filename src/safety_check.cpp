#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <cmath>
#include "swarm_robots/safety_check.h"

using std::string;

SafetyCheck::SafetyCheck(string ns, ros::NodeHandle* nh){
    this->nh_ = nh;
    this->sub_scan_ = this->nh_->subscribe("scan"+ns, 1, &Roam::ScanCallback, this);
    this->kfov_degrees = 180;
}

void SafetyCheck::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  this->laser_scan_ = *msg;
}


State SafetyCheck::GetSafeDirection(){
    double repel_lateral=0;
    double repel_drive=0;

    for (int i = 0; i < this->kfov_degrees_ / 2; i++) {
            repel_lateral += (1/scan.ranges[i])*sin(i * M_PI / 180);
            repel_lateral += (1/scan.ranges[359-i])*sin(i * M_PI / 180);
            
            repel_drive += 1/scan.ranges[i]*cos(i * M_PI / 180);
            repel_drive += 1/scan.ranges[359-i]*cos(i * M_PI / 180);
    }
  return State(repel_drive,repel_lateral);
}


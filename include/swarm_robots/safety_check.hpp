/**
 * @file safety_check.hpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for agent robot safety check
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_  //  NOLINT
#define INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <string>
#include <utility>
#include <vector>

#include "state.hpp"

using std::string;

class SafetyCheck {
   public:  //  NOLINT
    /**
     * @brief SafetyCheck parameterized constructor,
     * initializes agent namespace and nodehandle
     * @param string ns : agent namespace
     * @param ros::NodeHandle* nh : ros node handle
     */
    SafetyCheck(string ns, ros::NodeHandlePtr nh);

    /**
     * @brief Laser scan callback function
     * @param const sensor_msgs::LaserScan::ConstPtr& msg : msg pointer
     */
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief Gets safce obstacle free distance
     * @return State : x and y coordinates of safe distance from current
     */
    State GetSafeDirection();

   private:                              //  NOLINT
    double max_range_;                   ///< Range for obstacle avoiance
    double kfov_degrees_;                ///< fov degress = 180
    ros::Subscriber laser_sub_;          ///< Laser topic subscriber
    ros::NodeHandlePtr nh_;                ///< ros nodehandle
    sensor_msgs::LaserScan laser_scan_;  ///< Variable to store laser scan data
};
#endif  // INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_        //  NOLINT

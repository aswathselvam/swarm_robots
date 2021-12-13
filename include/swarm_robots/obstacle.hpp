/**
 * @file swarm_master.hpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for arena obstacles
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

#ifndef INCLUDE_SWARM_ROBOTS_OBSTACLE_HPP_
#define INCLUDE_SWARM_ROBOTS_OBSTACLE_HPP_

#include <string>
using std::string;

class Obstacle {
   public:  //  NOLINT
    /**
     * @brief AgentNode parameterized constructor,
     * initializes agent namespace
     * @param string ns : agent namespace
     */
    explicit Obstacle(string ns);

    /**
     * @brief Set values to Obstacle x,y, and dimensions
     * @param double x : new x value
     * @param double y : new y value
     */
    bool UpdateObstacleLocation(double x, double y);

    double x_;       ///< Obstacle x coordinate
    double y_;       ///< Obstacle y coordinate
    double length_;  ///< Obstacle length
    double width_;   ///< Obstacle width
};

#endif  // INCLUDE_SWARM_ROBOTS_OBSTACLE_HPP_

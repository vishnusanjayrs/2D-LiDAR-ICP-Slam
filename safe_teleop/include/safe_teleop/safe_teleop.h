/**
 * @file safe_teleop.h
 * Header for safe_teleop library
 * Created by rakesh on 28/09/18.
 */

#ifndef SAFE_TELEOP_SAFE_TELEOP_NODE_H
#define SAFE_TELEOP_SAFE_TELEOP_NODE_H

#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
// twist message with timestamp
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/String.h"

// boost includes for multi-threading
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/atomic.hpp>

// it's a good practice to create a library with its own namespace
namespace safe_teleop
{
class SafeTeleop
{
public:
  /**
   * @brief class constructor
   * @details Initializations required for the node (e.g. instantiations of publisher/subscriber objects etc)
   */
  SafeTeleop();

  /**
   * @brief class destructor
   * @details Publishes zero command velocity one last time so that the robot stops in case the command was non-zero
   */
  ~SafeTeleop();

  /**
   * @brief callback function on receiving new laser scan data
   * @details saves the most recent laser scan data in a thread-safe manner
   * @param laser_scan_msg (smart) pointer to the laser scan message
   */
  inline void laserScanCallback(const sensor_msgs::LaserScanConstPtr &laser_scan_msg)
  {
    boost::mutex::scoped_lock lock(laser_scan_mutex_);
    laser_scan_ = *laser_scan_msg;
    //ROS_WARN("I heard: [%s]", laser_scan_msg->header.c_str());
  }

  /**
   * @brief thread-safe getter method for laser_scan_ member
   * @return last received laser scan
   */
  inline sensor_msgs::LaserScan getLaserScan()
  {
    boost::mutex::scoped_lock lock(laser_scan_mutex_);
    return laser_scan_;
  }

  /**
   * @brief shutdown the teleop thread (run method)
   */
  void shutdown() { is_shutdown_ = true; }

  /**
   * @brief outputs current velocities to screen
   */
  void displayCurrentSpeeds() { ROS_INFO("Linear: %.2g, Angular: %.2g\r", (double)linear_speed_, (double)angular_speed_); }

  /**
  *
  * @tparam T float or double
  * @param normalised radians input
  * @return degrees
  
  template <class T>
  static T radians_degree(T angle)
  {
    auto out_degree = std::muliply(angle, 180/M_PI);
    return out_degree;
  }**/

  /**
  *
  * @tparam T float or double
  * @param angle angle to normalize (in radians)
  * @return normalize angle in the range (0, 2 * pi)
  */
  template <class T>
  static T normalizeTo360Angle(T angle)
  {
    auto out_angle = std::fmod(angle, 2*M_PI);
    if (out_angle < 0)
    {
      out_angle += 2*M_PI;
    }
    return out_angle;
  }

  /**
   *
   * @tparam T float or double
   * @param angle angle to normalize (in radians)
   * @return normalize angle in the range (-pi, pi)
   */
  template <class T>
  static T normalizeTo180Angle(T angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  // Implement the following methods

  /**
   * method that publishes command velocities at a fixed rate. Additional timeout to give 0 velocity
   * when no command is received
   */
  void run();

  // move commands
  void moveForward();
  void moveBackward();
  void rotateClockwise();
  void rotateCounterClockwise();
  void stop();

  void increaseLinearSpeed();   ///< Increase linear speed by 0.05 m/s
  void decreaseLinearSpeed();   ///< Decrease linear speed by 0.05 m/s
  void increaseAngularSpeed();  ///< Increase angular speed by 0.05 rad/s
  void decreaseAngularSpeed();  ///< Decrease angular speed by 0.05 rad/s

  /**
   * @brief check if the linear velocity is safe based on most recent laser scan
   * @param linear_vel linear velocity of the robot
   * @return whether the velocity is safe
   */
  bool checkSafety(double linear_vel);

  // any other method if needed

protected:
  // ROS Standard: end your attributes with underscore ('_')

  ros::Publisher cmd_vel_pub_;      ///< Publisher for command velocity topic
  ros::Subscriber laser_scan_sub_;  ///< Subscriber for laser scan topic

  sensor_msgs::LaserScan laser_scan_;   ///< Last received laser scan
  boost::mutex laser_scan_mutex_;       ///< Mutex for accessing laser scan member

  boost::atomic_bool is_shutdown_;      ///< Whether need to shutdown the teleop thread
  boost::thread run_thread_;            ///< Thread handler for the run method

  // current state of the tele-operation (atomic for thread-safety)
  // speed: only magnitude
  boost::atomic<double> linear_speed_;               ///< Current linear velocity
  boost::atomic<double> angular_speed_;              ///< Current angular velocity

  // velocity: magnitude + direction
  boost::atomic<double> linear_vel_;               ///< Current linear velocity
  boost::atomic<double> angular_vel_;              ///< Current angular velocity

  boost::atomic<double> last_command_timestamp_;   ///< Last time a command was received

  // parameters for tele-operation
  double linear_vel_increment_;         ///< Increment for linear vel
  double angular_vel_increment_;        ///< Increment for angular vel
  double max_cmd_vel_age_;              ///< Max age of cmd vel before timing out and publishing zero velocity
  double max_linear_vel_;               ///< Max linear velocity
  double max_angular_vel_;              ///< Max angular velocity
  double laser_safety_check_angle_;     ///< Angle of the laser scan to check for safety (in radians)
  double min_safety_impact_time_;       ///< Min time for impact to be considered to be safe
  double min_safety_distance_;          ///< Min distance consdiered to be safe

  // add other attributes as required

};
} // namespace safe_teleop_node

#endif //SAFE_TELEOP_SAFE_TELEOP_NODE_H

/**
 * @file safe_teleop.cpp
 * @brief Safe teleoperation library implementation
 * Created by rakesh on 28/09/18.
 */
#include <limits>
#include <safe_teleop/safe_teleop.h>

namespace safe_teleop
{

SafeTeleop::SafeTeleop() :
  is_shutdown_(false),
  max_cmd_vel_age_(1.0),
  max_linear_vel_(1.01),
  max_angular_vel_(1.01),
  linear_vel_increment_(0.05),
  angular_vel_increment_(0.05),
  laser_safety_check_angle_(0.25),
  min_safety_impact_time_(0.5),
  min_safety_distance_(0.5),
  linear_vel_(0.0),
  angular_vel_(0.0),
  last_command_timestamp_(0.0)
{
  ros::NodeHandle global_nh;
  cmd_vel_pub_ = global_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  // The subscriber callback is set to the laserScanCallback method of the instantiated object of this class
  laser_scan_sub_ = global_nh.subscribe("scan", 5, &SafeTeleop::laserScanCallback, this);

  run_thread_ = boost::thread(&SafeTeleop::run, this);
  displayCurrentSpeeds();
}

SafeTeleop::~SafeTeleop()
{
  shutdown();
  // wait for the run thread to terminate
  run_thread_.join();

  geometry_msgs::Twist zero_cmd_vel;
  zero_cmd_vel.linear.x = 0;
  zero_cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(zero_cmd_vel);
}

void SafeTeleop::run()
{
  ros::Rate r(10);

  while (ros::ok() && !is_shutdown_)
  {
    auto current_timestamp = ros::Time::now().toSec();

    auto last_cmd_vel_age = current_timestamp - last_command_timestamp_;

    if (last_cmd_vel_age > max_cmd_vel_age_)
    {
      geometry_msgs::Twist zero_cmd_vel;
      zero_cmd_vel.linear.x = 0;
      zero_cmd_vel.angular.z = 0;
      cmd_vel_pub_.publish(zero_cmd_vel);
    }
    else
    { 
      auto is_safe = checkSafety(static_cast<double>(linear_vel_));
      if (is_safe)
      {
        geometry_msgs::Twist safe_cmd_vel;
      safe_cmd_vel.linear.x = linear_vel_.load(boost::memory_order_relaxed);
      safe_cmd_vel.angular.z = angular_vel_.load(boost::memory_order_relaxed);
      displayCurrentSpeeds();
      cmd_vel_pub_.publish(safe_cmd_vel);
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "safety check triggerred\r");
      } 
      
    }

    r.sleep();
  }
}

void SafeTeleop::moveForward()
{ 
  last_command_timestamp_ = ros::Time::now().toSec();
  linear_vel_.store(linear_speed_.load(boost::memory_order_relaxed),boost::memory_order_relaxed); 
  angular_vel_.store(0.0);
}

void SafeTeleop::moveBackward()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  linear_vel_.store(-1.0*(linear_speed_.load(boost::memory_order_relaxed)),boost::memory_order_relaxed);
  angular_vel_.store(0.0);
}

void SafeTeleop::rotateClockwise()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  angular_vel_.store(-1.0* (angular_speed_.load(boost::memory_order_relaxed)),boost::memory_order_relaxed);
  linear_vel_.store(0.0); 
}

void SafeTeleop::rotateCounterClockwise()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  angular_vel_.store(angular_speed_.load(boost::memory_order_relaxed),boost::memory_order_relaxed);
  linear_vel_.store(0.0); 
}

void SafeTeleop::stop()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  linear_vel_.store(0.0,boost::memory_order_relaxed);
  angular_vel_.store(0.0,boost::memory_order_relaxed);
}


void SafeTeleop::increaseLinearSpeed()
{
    //ROS_WARN("Method not implemented\r");
  if ((double)((linear_speed_.load(boost::memory_order_relaxed))+linear_vel_increment_) > max_linear_vel_ )
  {
    ROS_WARN("Cannot increase linear speed above %f\r",max_linear_vel_);
  }
  else
  {
    linear_speed_.store((linear_speed_.load(boost::memory_order_relaxed))+linear_vel_increment_,boost::memory_order_relaxed);
  }
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseLinearSpeed()
{
  if ((linear_speed_.load(boost::memory_order_relaxed))-linear_vel_increment_ <= 0.0 )
  {
    ROS_WARN("Cannot decrease linear speed below 0.0\r");
  }
  else
  {
    linear_speed_.store((linear_speed_.load(boost::memory_order_relaxed))-linear_vel_increment_,boost::memory_order_relaxed);
  }
  displayCurrentSpeeds();
}

void SafeTeleop::increaseAngularSpeed()
{
  if ((angular_speed_.load(boost::memory_order_relaxed))+angular_vel_increment_ > max_angular_vel_ )
  {
    ROS_WARN("Cannot increase angular speed above %f\r",max_linear_vel_);
  }
  else
  {
    angular_speed_.store((angular_speed_.load(boost::memory_order_relaxed))+angular_vel_increment_,boost::memory_order_relaxed);
  }
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseAngularSpeed()
{
  if ((angular_speed_.load(boost::memory_order_relaxed))-angular_vel_increment_ <= 0.0 )
  {
    ROS_WARN("Cannot decrease angular speed below 0.0\r");
  }
  else
  {
    angular_speed_.store((angular_speed_.load(boost::memory_order_relaxed))-angular_vel_increment_,boost::memory_order_relaxed);
  }
  displayCurrentSpeeds();
}

bool SafeTeleop::checkSafety(double linear_vel)
{
  auto laser_scan = getLaserScan();
  auto range_size = (laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment ;
  auto angle_r = laser_scan.angle_min;
  auto norm_angle = laser_scan.angle_min;
  if (linear_vel_.load(boost::memory_order_relaxed) == 0.0) ///<safety check not required for turning
  {
    return true;
  }
  for (float i = 0 ; i<range_size ; i++)
  {
    angle_r = angle_r+laser_scan.angle_increment;
    if (laser_scan.angle_min != 0) ///< normalise when the angle is from -pi to +pi to 0 to 2pi
    {
      norm_angle = normalizeTo360Angle(angle_r);
    }
    else
    {
      norm_angle = angle_r;
    }
    
    if (linear_vel_.load(boost::memory_order_relaxed) > 0.0) ///<forward safety check
    {
      if ((norm_angle < (laser_safety_check_angle_)) || (norm_angle > (2*M_PI - laser_safety_check_angle_))) ///<forward angles 2*pi-0.25 to 0,25
      {
        auto m_dist = laser_scan.ranges[i]; ///<get range distance
        if (m_dist < min_safety_distance_) ///< if less than min_safety_distance return false
        {
          return false;
        }
      }
    }
    else // backward safety check 
    {
      if ((norm_angle > (M_PI - laser_safety_check_angle_)) && (norm_angle < (M_PI + laser_safety_check_angle_))) ///<backward angles
      {
        auto m_dist = laser_scan.ranges[i];
        if (m_dist < min_safety_distance_)
        {
          return false;
        }
      }
    } 
  }
  return true;
}

} // namespace safe_teleop_node



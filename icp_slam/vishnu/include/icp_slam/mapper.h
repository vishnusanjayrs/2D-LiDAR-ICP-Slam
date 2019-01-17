/** @file mapper.h */
//
// Created by rakesh on 27/08/18.
//

#ifndef ICP_SLAM_MAPPER_H
#define ICP_SLAM_MAPPER_H

// stdlib includes
#include <map>

// misc includes
#include <boost/thread/recursive_mutex.hpp>
#include <boost/atomic.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#include <omp.h>
#include <atomic>

namespace icp_slam
{

// cost values for different occupancy tyes
static const unsigned char NO_INFORMATION = -1;
static const unsigned char FREE_SPACE = 0;
static const unsigned char LETHAL_OBSTACLE = 100;

class Mapper
{
public:

  typedef struct
  {
    double x;
    double y;
    double a;
  } robot_pose_t;

  typedef boost::recursive_mutex mutex_t;

  Mapper();

  cv::Mat getMapCopy();

  int getWidth() { return width_; }

  int getHeight() { return height_; }

  bool isInitialized() { return is_initialized_; }

  mutex_t& getMutex() { return mutex_; }

  /**
   *
   * @param width width of map in meters
   * @param height height of map in meters
   * @param resolution meters/pixel in map
   * @param unknown_cost_value value to initialize the map with (the map is unknown at the beginning)
   */
  void initMap(int width, int height, float resolution,
               double origin_x_meters, double origin_y_meters,
               uint8_t *pointer=nullptr, unsigned char unknown_cost_value=NO_INFORMATION);

  // @return: 1 - success, 0 - failure
  int updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan,
                const tf::StampedTransform &pose);

  int updateLaserScan(const sensor_msgs::LaserScanConstPtr &laser_scan, robot_pose_t robot_pose);

  int drawScanLine(int x1, int y1, int x2, int y2);

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

  robot_pose_t getRobotPose() { return robot_pose_; }

  cv::Point2d getRobotpoints(robot_pose_t &init_pose,
							robot_pose_t &current_pose);

  // utilities
  /**
   * @brief convert coords from continuous world coordinate to discrete image coord
   */
  int convertToGridCoords(double x, double y, int &grid_x, int &grid_y);

  /**
   * @brief convert coords from continuous world coordinate to discrete image coord
   */
  int convertToWorldCoords(int grid_x, int grid_y, double &x, double &y);

  robot_pose_t poseFromGeometryPoseMsg(const geometry_msgs::Pose &pose_msg);
  robot_pose_t poseFromTf(const tf::StampedTransform &tf_pose);

  cv::Mat getLaserpoints(const sensor_msgs::LaserScanConstPtr &laser_scan_ptr,
                               const Mapper::robot_pose_t &current_pose);

protected:
  mutex_t mutex_;
  boost::atomic_bool is_initialized_;

  cv::Mat map_;
  cv::Mat relative_map_;

  int width_;
  int height_;
  float resolution_;  ///< @brief meters/pixels

  double origin_x_; ///< origin of the map in meters
  double origin_y_; ///< origin of the map in meters

  robot_pose_t robot_pose_;
  robot_pose_t init_robot_pose_;

  int init_robot_pt_x ;
  int init_robot_pt_y ;

  cv::Point2d init_robot_grid_pts;

};
} // namespace icp_slam

#endif //ICP_SLAM_MAPPER_H

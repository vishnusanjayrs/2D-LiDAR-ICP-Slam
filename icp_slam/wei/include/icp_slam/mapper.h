/** 
 * @file mapper.h
 *
 * Created by rakesh on 27/08/18.
 */

#ifndef ICP_SLAM_MAPPER_H
#define ICP_SLAM_MAPPER_H

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
  // Cost values for different occupancy tyes.
  static const unsigned char NO_INFORMATION = -1;
  static const unsigned char FREE_SPACE = 0;
  static const unsigned char LETHAL_OBSTACLE = 100;

  class Mapper
  {
    // TODO:
  };
}
#endif

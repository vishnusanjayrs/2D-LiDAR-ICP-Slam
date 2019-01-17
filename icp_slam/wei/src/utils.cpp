/**
 * Created by rakesh on 13/08/18.
 *
 * @file utils.cpp
 */

#include <icp_slam/utils.h>

namespace icp_slam
{
  namespace utils
  {
    cv::Mat laserScanToPointMat(
      const sensor_msgs::LaserScanConstPtr &laser_scan_ptr)
    {
      sensor_msgs::LaserScan scan = *laser_scan_ptr;
      int number_of_scan = 
        (int)((scan.angle_max - scan.angle_min) / scan.angle_increment);
      cv::Mat point_mat = cv::Mat(number_of_scan, 2, CV_32F);
      float x, y;
      auto teta = scan.angle_min;

      for (int i = 0; i < number_of_scan; i++)
      {
        teta += scan.angle_increment;
        if (scan.ranges[i] > scan.range_min && scan.ranges[i] < scan.range_max)
        {
          polarToCartesian(scan.ranges[i], teta , x, y);
        }
        else
        {
          x = 0.0;
          y = 0.0;
        }
        point_mat.at<float>(i, 0) = x;
        point_mat.at<float>(i, 1) = y;
      }
      return point_mat;
    }

    cv::Mat transformPointMat(tf::Transform transform, cv::Mat &point_mat)
    {
      assert(point_mat.data);
      assert(!point_mat.empty());

      cv::Mat point_mat_homogeneous(
      	3, point_mat.rows, CV_32F, cv::Scalar(1.0f));

      cv::Mat(point_mat.t()).copyTo(point_mat_homogeneous.rowRange(0, 2));

      auto T = transformToMatrix(transform);
      cv::Mat transformed_point_mat =  T * point_mat_homogeneous;
      return cv::Mat(transformed_point_mat.t()).colRange(0, 2);
    }
  }
}

//
// Created by rakesh on 15/08/18.
//

/** @file utils.h
 * @brief commonly used utilites
 */
#ifndef ICP_SLAM_UTILS_H
#define ICP_SLAM_UTILS_H

#include <numeric>
#include <algorithm>

#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#include <opencv2/opencv.hpp>

namespace icp_slam
{

namespace utils
{
/**
 *
 * @param r
 * @param a
 * @param x
 * @param y
 */
inline void polarToCartesian(float r, float a, float &x, float &y)
{
  x = r * std::cos(a);
  y = r * std::sin(a);
}

/**
 *
 * @tparam T
 * @param degree
 * @return
 */
template <class T>
inline T degreeToRadian(T degree) { return degree * M_PI / 180.0; }

/**
 *
 * @tparam T
 * @param radian
 * @return
 */
template <class T>
inline T radianToDegree(T radian) { return radian * 180.0 / M_PI; }

/**
 *
 * @tparam T numeric type (usually float or double)
 * @param vec
 * @param mean
 * @param std_dev
 */
template<class T>
void meanAndStdDev(const std::vector<T> &vec, T &mean, T &std_dev)
{
  size_t sz = vec.size();
  if (sz == 1)
  {
    mean = vec[0];
    std_dev = 0;
    return;
  }

  // Calculate the mean
  mean = std::accumulate(vec.begin(), vec.end(), static_cast<T>(0.0)) / sz;

  T variance =  std::accumulate(
    vec.begin(), vec.end(), static_cast<T>(0.0),
    [&mean](T accumulator, const T& val) {
      return accumulator + ((val - mean)*(val - mean));
    }
  );

  std_dev = std::sqrt(variance) / sz;
}

template <class T>
void findQuartiles(std::vector<T> vec, T &first_quartile, T &second_quartile, T &third_quartile)
{
  auto Q1 = vec.size() / 4;
  auto Q2 = vec.size() / 2;
  auto Q3 = Q1 + Q2;

  std::nth_element(vec.begin(),          vec.begin() + Q1, vec.end());
  std::nth_element(vec.begin() + Q1 + 1, vec.begin() + Q2, vec.end());
  std::nth_element(vec.begin() + Q2 + 1, vec.begin() + Q3, vec.end());

  first_quartile = vec[Q1];
  second_quartile = vec[Q2];
  third_quartile = vec[Q3];
}


/**
 * @brief convert 3D tf transform to 2D transformation matrix (ignoring z)
 * @param transform 3D tf transformation
 * @return 3x3 (2D) transformation matrix
 */
inline cv::Mat transformToMatrix(tf::Transform transform)
{
  cv::Mat matrix(cv::Mat::eye(3, 3, CV_32F));
  matrix.at<float>(0, 2) = transform.getOrigin().getX();
  matrix.at<float>(1, 2) = transform.getOrigin().getY();

  auto rotation_matrix_3d = transform.getBasis();
  matrix.at<float>(0, 0) = rotation_matrix_3d[0][0];
  matrix.at<float>(0, 1) = rotation_matrix_3d[0][1];
  matrix.at<float>(1, 0) = rotation_matrix_3d[1][0];
  matrix.at<float>(1, 1) = rotation_matrix_3d[1][1];

  return matrix;
}

/**
 *
 * @param scan laser scan
 * @return Nx2 CV matrix
 */
cv::Mat laserScanToPointMat(const sensor_msgs::LaserScanConstPtr &scan);

/**
 *
 * @param transform transformation for the point cloud
 * @param point_mat Nx2 CV matrix of 2D points
 * @return Nx2 CV matrix of transformed point cloud
 */
cv::Mat transformPointMat(tf::Transform transform, cv::Mat &point_mat);

} // namespace utils

} // namespace icp_slam

#endif //ICP_SLAM_UTILS_H

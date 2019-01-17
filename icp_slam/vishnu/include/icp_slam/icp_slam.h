//
// Created by rakesh on 13/08/18.
//

/** @file icp_slam.h
 * @brief declaration for the ICPSlam class
 * @todo remove repeated polar to cartesian conversions!!!!
 */

#ifndef ICP_SLAM_ICP_SLAM_H
#define ICP_SLAM_ICP_SLAM_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#include <opencv2/opencv.hpp>

namespace icp_slam
{

/**
 * @brief class for ICP-SLAM.
 *  Should be largely independent of ROS (except for basic data structures for sensors and poses)
 */
class ICPSlam
{
public:
  /**
   *
   * @param min_keyframes_distance min distance between two keyframes
   * @param min_keyframes_angle min angle between two keyframes
   * @param max_keyframes_time max time separation between two frames
   */
  ICPSlam(tfScalar max_keyframes_distance, tfScalar max_keyframes_angle, double max_keyframes_time);

  /**
   * @brief track the current laser frame
   * @param laser_scan laser data
   * @param current_frame_tf_odom_laser pose (transform) of sensor in odometry frame
   * @param tf_map_laser pose in the map frame
   * @return if a new keyframe was created
   */
   bool track(const sensor_msgs::LaserScanConstPtr &laser_scan,
              const tf::StampedTransform &current_frame_tf_odom_laser,
              tf::StampedTransform &tf_map_laser);

  /**
   *
   * @param current_frame_tf current sensor pose
   * @param last_kf_tf last keyframe sensor pose
   * @return whether to create a new keyframe
   */
  bool isCreateKeyframe(const tf::StampedTransform &current_frame_tf, const tf::StampedTransform &last_kf_tf) const;

  /**
   *
   * @param laser_scan1
   * @param laser_scan2
   * @param T_2_1 estimated transform from scan2 to scan1 (T_scan2_scan1)
   * @return refined T_scan2_scan1
   */
  static tf::Transform icpRegistration(const cv::Mat &last_point_mat,
                                       const cv::Mat &curr_point_mat,
                                       const tf::Transform &T_2_1);



  /**
   *
   * @param point_mat1 nx2 matrix with points of laser scan 1 with a match in laser scan 2
   * @param point_mat2 nx2 matrix with points of laser scan 2 with a match in laser scan 1
   * @return estimated transform from scan2 to scan1 (T_scan2_scan1) that minimizes least square alignment error
   */
  static tf::Transform icpIteration(cv::Mat &point_mat1,
                                    cv::Mat &point_mat2,
                                    cv::Mat &x_mean,
                                    cv::Mat &p_mean
                                    );

  /**
   * @brief find closest indices in two matrix of 2d points (in the same coordinate frame)
   * @param point_mat1
   * @param point_mat2
   * @param closest_indices index of closest points from scan1 to scan2, -1 for invalid
   * @param dists square of distances of the correspondences
   * @todo to be replaced by k-d tree based methods
   */
  static void closestPoints(cv::Mat &point_mat1,
                            cv::Mat &point_mat2,
                            std::vector<int> &closest_indices,
                            std::vector<float> &closest_distances_2);

  /**
   * @brief visualize the point cloud closest points
   * @param point_mat1 nx2 matrix with points of laser scan 1
   * @param point_mat2 nx2 matrix with points of laser scan 2
   * @param T_2_1 estimated transform from scan1 to scan2 (T_scan2_scan1)
   * @param closest_indices index of closest points from scan1 to scan2, -1 for invalid
   */
  static void vizClosestPoints(cv::Mat &point_mat1,
                               cv::Mat &point_mat2,
                               const tf::Transform &T_2_1,
                               int iteration);



protected:
  cv::Mat last_kf_pc_mat_;     ///< last pointcloud matrix
  tf::StampedTransform last_kf_tf_odom_laser_;    ///< Transform in odom frame of last keyframe
  tf::StampedTransform last_kf_tf_map_laser_;     ///< Transform in map frame of last keyframe


  tfScalar max_keyframes_distance_;
  tfScalar max_keyframes_angle_;
  double max_keyframes_time_;

  boost::atomic_bool is_tracker_running_;

  //unsigned int keyframe_count_ = 0;

}; // class ICPSlam
} // namespace icp_slam
#endif //ICP_SLAM_ICP_SLAM_H

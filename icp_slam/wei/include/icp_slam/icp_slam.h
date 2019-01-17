/**
 * Created by rakesh on 13/08/18.
 *
 * @file icp_slam.h
 * @brief Declaration for the ICPSlam class.
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
   * @brief Class for ICP-SLAM.
   * Should be largely independent of ROS (except for basic data structures for
   * sensors and poses).
   */
  class ICPSlam
  {
    public:
    /**
     *
     * @param min_keyframes_distance Min distance between two keyframes.
     * @param min_keyframes_angle Min angle between two keyframes.
     * @param max_keyframes_time Max time separation between two frames.
     */
    ICPSlam(
      tfScalar max_keyframes_distance, 
      tfScalar max_keyframes_angle, 
      double max_keyframes_time
    );

    /**
     * @brief Track the current laser frame.
     * @param laser_scan Laser data.
     * @param current_frame_tf_odom_laser Transform of sensor in odome frame.
     * @param tf_map_laser Pose in the map frame.
     * @return If a new keyframe was created.
     */
     bool track(
      const sensor_msgs::LaserScanConstPtr &laser_scan,
      const tf::StampedTransform &current_frame_tf_odom_laser,
      tf::StampedTransform &tf_map_laser
    );

    /**
     *
     * @param current_frame_tf Current sensor pose.
     * @param last_kf_tf Last keyframe sensor pose.
     * @return Whether to create a new keyframe.
     */
    bool isCreateKeyframe(
      const tf::StampedTransform &current_frame_tf, 
      const tf::StampedTransform &last_kf_tf
    ) const;

    /**
     *
     * @param dst_point_mat
     * @param src_point_mat
     * @param trans Estimated transform from scan2 to scan1 (T_scan2_scan1).
     * @return Refined T_scan2_scan1.
     */
    static tf::Transform icpRegistration(
      const cv::Mat &dst_point_mat,
      const cv::Mat &src_point_mat,
      const tf::Transform &trans
    );

    /**
     *
     * @param point_mat1 Nx2 matrix with points of scan 1 with a match in 2.
     * @param point_mat2 Nx2 matrix with points of scan 2 with a match in 1.
     * @return Estimated transform from scan2 to scan1 (T_scan2_scan1) that
     * minimizes least square alignment error.
     */
    static tf::Transform icpIteration(
      cv::Mat &point_mat1,
      cv::Mat &point_mat2,
      cv::Mat &x_mean,
      cv::Mat &p_mean
    );

    /**
     *
     * @brief Find closest indices in two matrix of points in same coordinate.
     * @param point_mat1
     * @param point_mat2
     * @param closest_indices Index of closest points from scan1 to scan2, -1 
     * for invalid.
     * @param dists Square of distances of the correspondences.
     */
    static void closestPoints(
      cv::Mat &point_mat1,
      cv::Mat &point_mat2,
      std::vector<int> &closest_indices,
      std::vector<float> &closest_distances_2
    );

    /**
     *
     * @brief Visualize the point cloud closest points.
     * @param point_mat1 Nx2 matrix with points of laser scan 1.
     * @param point_mat2 Nx2 matrix with points of laser scan 2.
     * @param T_2_1 Estimated transform from scan1 to scan2 (T_scan2_scan1).
     * @param closest_indices Index of closest points from scan1 to scan2, -1
     * @param file_path Path the visulization will be stored.
     * for invalid.
     */
    static void vizClosestPoints(
      cv::Mat &original_point_mat1,
      cv::Mat &original_point_mat2,
      const tf::Transform &trans,
      std::string file_path = "/tmp/icp_slam/viz_closest_points.png"
    );

  protected:
    // Laser scan of last kf.
    sensor_msgs::LaserScanPtr last_kf_laser_scan_;

    // Transform in odom frame of last keyframe
    tf::StampedTransform last_kf_tf_odom_laser_;

    // Transform in map frame of last keyframe
    tf::StampedTransform last_kf_tf_map_laser_;

    tfScalar max_keyframes_distance_;
    tfScalar max_keyframes_angle_;
    double max_keyframes_time_;
    boost::atomic_bool is_tracker_running_;
  };
}
#endif

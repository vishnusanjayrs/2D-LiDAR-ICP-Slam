/**
 * Created by rakesh on 13/08/18.
 *
 * @file icp_slam.cpp
 */

#include <cmath>
#include <map>
#include <numeric>
#include <chrono>

#include <boost/filesystem.hpp>
#include <boost/atomic/atomic.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/utils.h>

#define TIME_DIFF(tic, toc) \
  ((std::chrono::duration<double, std::milli>((toc) - (tic))).count())

namespace icp_slam
{

  ICPSlam::ICPSlam(
    tfScalar max_keyframes_distance, 
    tfScalar max_keyframes_angle, 
    double max_keyframes_time): 
      max_keyframes_distance_(max_keyframes_distance),
      max_keyframes_angle_(max_keyframes_angle),
      max_keyframes_time_(max_keyframes_time),
      last_kf_laser_scan_(new sensor_msgs::LaserScan()),
      is_tracker_running_(false)
  {
    last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
  }

  bool ICPSlam::track(
    const sensor_msgs::LaserScanConstPtr &laser_scan,
    const tf::StampedTransform &current_frame_tf_odom_laser,
    tf::StampedTransform &tf_map_laser)
  {
    if (is_tracker_running_)
    {
      ROS_WARN_THROTTLE(1.0, "Couldn't track frame, tracker already running.");
      return false;
    }

    // TODO: find the pose of laser in map frame.
    // If a new keyframe is created, run ICP.
    // If not a keyframe, obtain the laser pose in map frame based on odometry
    // update.
  }

  bool ICPSlam::isCreateKeyframe(
    const tf::StampedTransform &current_frame_tf, 
    const tf::StampedTransform &previous_frame_tf) const
  {
    assert(
      current_frame_tf.frame_id_ == previous_frame_tf.frame_id_);
    assert(
      current_frame_tf.child_frame_id_ == previous_frame_tf.child_frame_id_);

    float current_x = current_frame_tf.getOrigin().getX();
    float current_y = current_frame_tf.getOrigin().getY();
    float current_rotation = 
      tf::getYaw(current_frame_tf.getRotation()) * 180 / M_PI;

    float previous_x = previous_frame_tf.getOrigin().getX();
    float previous_y = previous_frame_tf.getOrigin().getY();
    float previous_rotation = 
      tf::getYaw(previous_frame_tf.getRotation()) * 180 / M_PI;

    float distance = 
      sqrt(pow((previous_x - current_x), 2) + 
        pow((previous_y - current_y), 2));
    float rotation = abs(previous_rotation - current_rotation);
   
    if ((distance > max_keyframes_distance_) || 
        (rotation > max_keyframes_angle_))
    {
      return true;
    }
    return false;
  }

  void ICPSlam::closestPoints(
    cv::Mat &point_mat1,
    cv::Mat &point_mat2,
    std::vector<int> &closest_indices,
    std::vector<float> &closest_distances)
  {
    // Uses FLANN for K-NN e.g. http://www.morethantechnical.com/2010/06/06/
    // iterative-closest-point-icp-with-opencv-w-code/
    closest_indices = std::vector<int>(point_mat1.rows, -1);
    closest_distances = std::vector<float>(point_mat1.rows, -1);


    cv::Mat multi_channeled_mat1;
    cv::Mat multi_channeled_mat2;

    point_mat1.convertTo(multi_channeled_mat1, CV_32FC2);
    point_mat2.convertTo(multi_channeled_mat2, CV_32FC2);

    cv::flann::Index flann_index(
      multi_channeled_mat2,
      cv::flann::KDTreeIndexParams(2));  // Using 2 randomized kdtrees.

    cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);
    cv::Mat mat_dists(point_mat1.rows, 1, CV_32F);
    flann_index.knnSearch(
      multi_channeled_mat1, 
      mat_indices, mat_dists, 
      1, 
      cv::flann::SearchParams(64));

    int* indices_ptr = mat_indices.ptr<int>(0);
    for (int i = 0; i < mat_indices.rows; ++i) {
      closest_indices[i] = indices_ptr[i];
    }

    mat_dists.copyTo(cv::Mat(closest_distances));

    // --------------------------- Naive Version --------------------------- //
    // Max allowed distance between corresponding points
    // const float max_distance = 0.5;
    //
    // for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
    // {
    //   int closest_point_idx = -1;
    //   float closest_distance_2 = std::pow(max_distance, 2.0f);
    //
    //   for (size_t j = 0, len_j = (size_t)point_mat2.rows; j < len_j; j++)
    //   {
    //     auto distance2 =
    //       std::pow(
    //         point_mat2.at<float>(j, 0) - point_mat1.at<float>(i, 0), 2.0f) + 
    //       std::pow(
    //         point_mat2.at<float>(j, 1) - point_mat1.at<float>(i, 1), 2.0f);
    //
    //     if (distance2 < closest_distance_2)
    //     {
    //       closest_distance_2 = distance2;
    //       closest_point_idx = (int)j;
    //     }
    //   }
    //
    //   if (closest_point_idx >= 0)
    //   {
    //     closest_indices[i] = closest_point_idx;
    //     closest_distances_2[i] = closest_distance_2;
    //   }
    // }
  }

  void ICPSlam::vizClosestPoints(
    cv::Mat &original_point_mat1,
    cv::Mat &original_point_mat2,
    const tf::Transform &trans,
    std::string file_path)
  {
    assert(original_point_mat1.size == original_point_mat2.size);

    // Transform the second matrix. One more dimension is added!
    cv::Mat point_mat2 = 
      utils::transformPointMat(trans.inverse(), original_point_mat2);
    cv::Mat point_mat1 = original_point_mat1;

    const float resolution = 0.005;

    // Compute canvas size.
    float min_x = point_mat1.at<float>(0, 0);
    float min_y = point_mat1.at<float>(0, 1);
    float max_x = point_mat1.at<float>(0, 0);
    float max_y = point_mat1.at<float>(0, 1);

    for (int i = 0; i < (size_t)point_mat1.rows; i++)
    {
      min_x = 
        point_mat1.at<float>(i, 0) < min_x ? 
        point_mat1.at<float>(i, 0) : min_x;
      min_y = 
        point_mat1.at<float>(i, 1) < min_y ? 
        point_mat1.at<float>(i, 1) : min_y;
      max_x = 
        point_mat1.at<float>(i, 0) > max_x ? 
        point_mat1.at<float>(i, 0) : max_x;
      max_y = 
        point_mat1.at<float>(i, 1) > max_y ? 
        point_mat1.at<float>(i, 1) : max_y;

      min_x = 
        point_mat2.at<float>(i, 0) < min_x ? 
        point_mat2.at<float>(i, 0) : min_x;
      min_y = 
        point_mat2.at<float>(i, 1) < min_y ? 
        point_mat2.at<float>(i, 1) : min_y;
      max_x = 
        point_mat2.at<float>(i, 0) > max_x ? 
        point_mat2.at<float>(i, 0) : max_x;
      max_y = 
        point_mat2.at<float>(i, 1) > max_y ? 
        point_mat2.at<float>(i, 1) : max_y;
    }

    // Create the image.
    int img_size = 1024;
    float padding = 0.1;
    cv::Mat img(
      (int)(img_size * (1 - padding)), 
      (int)(img_size * (1 - padding)), 
      CV_8UC3, cv::Scalar(0, 0, 0));

    // Translate both matrices to fit the canvas.
    float content_size = 0;
    float resize_ratio = 0;
    float x_translate = 0;
    float y_translate = 0;
    tf::Transform t;
    if (max_x - min_x > max_y - min_y)
    {
      content_size = max_x - min_x;
      x_translate = min_x;
      y_translate = min_y + (max_y - min_y + content_size) / 2.0;

    } 
    else 
    {
      content_size = max_y - min_y;
      x_translate = min_x + (max_x - min_x - content_size) / 2.0;
      y_translate = max_y;
    }

    t.setOrigin(tf::Vector3(x_translate, y_translate, 0.0));
    t.setRotation(tf::Quaternion(0, 0, 0, 1));
    resize_ratio = img_size * (1 - padding) / content_size;
    cv::circle(img, 
      cv::Point(-x_translate * resize_ratio, y_translate * resize_ratio), 
      5, cv::Scalar(0, 255, 0), -1);
    cv::line(img, 
      cv::Point(-x_translate * resize_ratio - 10, y_translate * resize_ratio), 
      cv::Point(-x_translate * resize_ratio + 10, y_translate * resize_ratio), 
      cv::Scalar(0, 255, 0), 2);
    cv::line(img, 
      cv::Point(-x_translate * resize_ratio, y_translate * resize_ratio - 10), 
      cv::Point(-x_translate * resize_ratio, y_translate * resize_ratio + 10), 
      cv::Scalar(0, 255, 0), 2);

    // Scale both matrices.
    point_mat1 = utils::transformPointMat(t.inverse(), point_mat1);
    point_mat2 = utils::transformPointMat(t.inverse(), point_mat2);
    point_mat1 = point_mat1.mul(resize_ratio);
    point_mat2 = point_mat2.mul(resize_ratio);

    // Plot points from both matrices.
    for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
    {
      float x1 = point_mat1.at<float>(i, 0);
      float y1 = -point_mat1.at<float>(i, 1);
      float x2 = point_mat2.at<float>(i, 0);
      float y2 = -point_mat2.at<float>(i, 1);

      cv::Point point1(x1, y1);
      cv::Point point2(x2, y2);

      cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1);
      cv::circle(img, point2, 5, cv::Scalar(255, 0, 0), -1);

      cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 2);
    }

    // Add padding and save results.
    cv::Mat img_pad(img_size, img_size, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::copyMakeBorder(
      img, img_pad, 
      (int)(img_size * padding), 
      (int)(img_size * padding), 
      (int)(img_size * padding), 
      (int)(img_size * padding), 
      cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::Mat tmp;
    cv::flip(img_pad, tmp, 0);
    boost::filesystem::path dir("/tmp/icp_slam");
    boost::filesystem::create_directory(dir);
    cv::imwrite(file_path, img_pad);
  }

  tf::Transform ICPSlam::icpRegistration(
    const cv::Mat &src_point_mat,
    const cv::Mat &dst_point_mat,
    const tf::Transform &trans)
  {
    assert(dst_point_mat.size == src_point_mat.size);
    float threshold = 0.00000001;
    float previous_sum = 0;
    int max_iteration = 50;
    cv::Mat src_original = src_point_mat.clone();
    cv::Mat dst_original = dst_point_mat.clone();
    cv::Mat src = src_point_mat.clone();
    cv::Mat dst = dst_point_mat.clone();
    tf::Transform final_trans = trans;

    std::vector<int> closest_indices;
    std::vector<float> closest_distances;

    for (int counter = 0; counter < max_iteration; counter++)
    {
      // Apply transformation first.
      src = utils::transformPointMat(final_trans, src_original);
      cv::Mat dst = dst_point_mat.clone();

      // Find correspondences and set to source.
      ICPSlam::closestPoints(src, dst, closest_indices, closest_distances);
      for (int i = 0; i < (size_t)src.rows; i++)
      {
        if (closest_indices.at(i) != i)
        {
          dst.at<float>(i, 0) = 
            dst_original.at<float>(closest_indices.at(i), 0);
          dst.at<float>(i, 1) = 
            dst_original.at<float>(closest_indices.at(i), 1);
        }
      }

      // Compute center of mass.
      cv::Scalar src_mean_x = cv::mean(src.col(0));
      cv::Scalar src_mean_y = cv::mean(src.col(1));
      cv::Scalar dst_mean_x = cv::mean(dst.col(0));
      cv::Scalar dst_mean_y = cv::mean(dst.col(1));

      cv::Mat src_mean = cv::Mat(src.rows, src.cols, CV_32F);
      src_mean.col(0).setTo(src_mean_x);
      src_mean.col(1).setTo(src_mean_y);

      cv::Mat dst_mean = cv::Mat(dst.rows, dst.cols, CV_32F);
      dst_mean.col(0).setTo(dst_mean_x);
      dst_mean.col(1).setTo(dst_mean_y);

      cv::Mat x_mean = cv::Mat(2, 1, CV_32F);
      cv::Mat p_mean = cv::Mat(2, 1, CV_32F);
      x_mean.at<float>(0, 0) = src_mean_x[0];
      x_mean.at<float>(1, 0) = src_mean_y[0];
      p_mean.at<float>(0, 0) = dst_mean_x[0];
      p_mean.at<float>(1, 0) = dst_mean_y[0];

      // Normailize source and destination.
      cv::Mat src_normalized = src - src_mean;
      cv::Mat dst_normalized = dst - dst_mean;

      // Perform ICP.
      tf::Transform predicted_trans = 
        icpIteration(src_normalized, dst_normalized, x_mean, p_mean);

      // Transform original source and compare results.
      cv::Mat src_transformed = 
        utils::transformPointMat(predicted_trans, src);
      ICPSlam::closestPoints(
        src_transformed, dst_original, closest_indices, closest_distances);

      final_trans *= predicted_trans;
      float sum = std::accumulate(
        closest_distances.begin(), closest_distances.end(), 0.0);
      float error = std::abs(sum - previous_sum) / src.rows;
      previous_sum = sum;

      // Save intermediate images.
      // std::stringstream ss;
      // ss << "/tmp/icp_slam/" << std::to_string(counter) << ".png";
      // vizClosestPoints(src_original, dst_original, final_trans, ss.str());
      
      if (error < threshold)
      {
        return final_trans;
      }
    }

    ROS_INFO("Max ICP Iteration Reached!\r");
    return final_trans;
  }

  tf::Transform ICPSlam::icpIteration(
    cv::Mat &src, 
    cv::Mat &dst, 
    cv::Mat &x_mean,
    cv::Mat &p_mean)
  {
    // Compute the W matrix.
    cv::Mat t;
    cv::transpose(dst, t);
    cv::Mat w = t * src;

    // Decompose W and compute rotation and translation.
    cv::SVD svd(w);
    cv::Mat rotation = svd.u * svd.vt;
    cv::Mat translation = x_mean - (rotation * p_mean);

    // Return the transformation.
    tf::Transform trans;
    trans.setOrigin(tf::Vector3(
      -translation.at<float>(0), -translation.at<float>(1), 0.0));

    auto rotation_angle = 
      std::atan2(rotation.at<float>(1, 0), rotation.at<float>(0, 0));
    trans.setRotation(tf::createQuaternionFromYaw(rotation_angle));

    return trans;
  }
}

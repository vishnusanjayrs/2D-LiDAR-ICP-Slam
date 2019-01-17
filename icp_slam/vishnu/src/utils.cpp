//
// Created by rakesh on 17/08/18.
//

#include <icp_slam/utils.h>
#include <iostream> 
using namespace std;

namespace icp_slam
{
namespace utils
{

cv::Mat laserScanToPointMat(const sensor_msgs::LaserScanConstPtr &laser_scan_ptr)
{
  //get ranges array from laserscan
  sensor_msgs::LaserScan laser_scan = *laser_scan_ptr;
	auto range_size = (laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment ;
	int row = (int)range_size;
	int col = 2;
	cv::Mat point_mat = cv::Mat(row,col,CV_32F);
  float x, y;
  auto teta = laser_scan.angle_min;
  //int row_cnt=0;
  for(int i=0;i<range_size;i++)
  {
    teta = teta + laser_scan.angle_increment;
    ///ROS_INFO("ranges : (%i),angle %f  is %f ",i,teta,laser_scan.ranges[i]);
    if((laser_scan.ranges[i]>laser_scan.range_min)&&(laser_scan.ranges[i]<laser_scan.range_max))
    {
    polarToCartesian(laser_scan.ranges[i], teta , x, y);
    //row_cnt++;
    }
    else
    {
      x=0.0;
      y=0.0;
    }
    point_mat.at<float>(i, 0)=x;
    point_mat.at<float>(i, 1)=y;
    
  }
  // cv::Mat point_mat = cv:Mat(row_cnt,col,CV_32F);
  // for(int i=0;i<range_size;i++)
  // {
  //   if temp[i][0]==0.0
  //   {
  //     continue;
  //   }
  //   point_mat.at<float>(i, 0)=temp[i][0];
  //   point_mat.at<float>(i, 1)=temp[i][1];
  // }
  ///cout<< point_mat << endl;
  return point_mat;
}

cv::Mat transformPointMat(tf::Transform transform, cv::Mat &point_mat)
{
  assert(point_mat.data);
  assert(!point_mat.empty());

  cv::Mat point_mat_homogeneous(3, point_mat.rows, CV_32F, cv::Scalar(1.0f));

  cv::Mat(point_mat.t()).copyTo(point_mat_homogeneous.rowRange(0, 2));

  auto T = transformToMatrix(transform);
  cv::Mat transformed_point_mat =  T * point_mat_homogeneous;
  return cv::Mat(transformed_point_mat.t()).colRange(0, 2);
}

} // namespace utils
} // namespace icp_slam
//
// Created by rakesh on 13/08/18.
//

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>

#include <icp_slam/icp_slam.h>
#include <icp_slam/mapper.h>
#include <string>
using namespace std;

using namespace icp_slam;

/**
 * @brief ROS interface to use ICP-SLAM from icp_slam.h
 */
class ICPSlamNode
{
public:
  ICPSlamNode();

  void laserCallback(const sensor_msgs::LaserScanConstPtr &laser_msg);
  void publishMap(ros::Time timestamp);

protected:

  ros::NodeHandle global_nh_;        ///< global namespace ROS handler
  ros::NodeHandle local_nh_;         ///< local namespace ROS handler
  tf::TransformListener tf_listener_; ///< ROS tf (transform) listener
  tf::TransformBroadcaster tf_broadcaster_;  ///< ROS tf broadcaster

  ros::Subscriber laser_sub_;         ///< Laser topic subscriber

  ros::Publisher map_publisher_;        ///< Occupancy map publisher
  ros::Duration map_publish_interval_;  ///< Time interval to publish map

  std::string odom_frame_id_;         ///< odometry frame id
  std::string map_frame_id_;          ///< map frame id

  boost::shared_ptr<ICPSlam> icp_slam_;

  double x_min_;
  double y_min_;
  double x_max_;
  double y_max_;

  double origin_x_;
  double origin_y_;

  unsigned int width_;    ///< @brief width of the map
  unsigned int height_;   ///< @brief height of the map

  double resolution_;
  Mapper mapper_;
  nav_msgs::OccupancyGrid occupancy_grid_;

  unsigned int keyframe_count_ = 0;

  boost::mutex laser_scan_mutex_;
  sensor_msgs::LaserScan laser_scan_; 

  ros::Time last_map_update=ros::Time::now();

};

ICPSlamNode::ICPSlamNode() : local_nh_("~")
{
  laser_sub_ = global_nh_.subscribe("scan", 10, &ICPSlamNode::laserCallback, this);
  map_publisher_ = local_nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);

  // getting ROS parameters:
  // local_nh_.param<TYPE>(PARAM_NAME, OUTPUT_VARIABLE, DEFAULT_VALUE)

  double map_update_interval;
  local_nh_.param("map_update_interval", map_update_interval, 5.0);
  map_publish_interval_.fromSec(map_update_interval);

  local_nh_.param<std::string>("odom_frame", odom_frame_id_, "odom");
  local_nh_.param<std::string>("map_frame", map_frame_id_, "map");

  // SLAM parameterss
  double max_keyframes_distance;
  double max_keyframes_angle;
  double max_keyframes_time;
  local_nh_.param<double>("max_keyframes_distance", max_keyframes_distance, 0.02);
  local_nh_.param<double>("max_keyframes_angle", max_keyframes_angle, 0.01);
  local_nh_.param<double>("max_keyframes_time", max_keyframes_time, 0.5);

  local_nh_.param("resolution",  resolution_, 0.05);

  local_nh_.param("xmin", x_min_, -15.0);
  local_nh_.param("ymin", y_min_, -15.0);
  local_nh_.param("xmax", x_max_, 15.0);
  local_nh_.param("ymax", y_max_, 15.0);

  // TODO: initialize the map (width_, height_, origin_)
  width_ = (unsigned int)((x_max_-x_min_)/resolution_);
  height_=(unsigned int)((y_max_- y_min_)/resolution_);

  origin_x_ = (x_max_ + x_min_) / 2;
  origin_y_ = (y_max_ + y_min_) / 2;

  icp_slam_.reset(new ICPSlam(max_keyframes_distance, max_keyframes_angle, max_keyframes_time));

  // occupancy grid
  occupancy_grid_.data.resize(width_ * height_);
  occupancy_grid_.header.frame_id = map_frame_id_;
  occupancy_grid_.info.width = width_;
  occupancy_grid_.info.height = height_;
  occupancy_grid_.info.resolution = resolution_;
  occupancy_grid_.info.origin.position.x = x_min_;
  occupancy_grid_.info.origin.position.y = y_min_;
  occupancy_grid_.info.origin.orientation.w = 1;

  mapper_.initMap(width_,height_,resolution_,origin_x_,origin_y_);

  

}

void ICPSlamNode::laserCallback(const sensor_msgs::LaserScanConstPtr &laser_msg)
{

  static ros::Time current_time(0,0);

  // TODO: get laser pose in odom frame (using tf)
  tf::StampedTransform tf_odom_laser;
  try
  {
    tf_listener_.lookupTransform(odom_frame_id_,laser_msg->header.frame_id,current_time, tf_odom_laser);
    // current pose
    tf::StampedTransform tf_map_laser;
    auto is_keyframe = icp_slam_->track(laser_msg, tf_odom_laser, tf_map_laser);
  
  // current pose
  //tf::StampedTransform tf_map_laser;

    //auto is_keyframe = icp_slam_->track(laser_msg, tf_odom_laser, tf_map_laser);
    if (is_keyframe)
    {
      //TODO: update the map
      //tf::StampedTransform stf_map_laser=tf::StampedTransform(tf_map_laser,ros::Time::now(),map_frame_id_,laser_msg->header.frame_id);
      mapper_.updateMap(laser_msg, tf_map_laser);
      
      //tf_broadcaster_.sendTransform(stf_map_laser);
      if (laser_msg->header.stamp - last_map_update > map_publish_interval_)
      {
        publishMap(laser_msg->header.stamp);
        last_map_update=ros::Time::now();
      }
    }

    

    // TODO: broadcast odom to map transform (using tf)
    tf::Transform tf_odom_map = tf_map_laser*tf_odom_laser.inverse();
    tf::StampedTransform stf_odom_map=tf::StampedTransform(tf_odom_map,ros::Time::now(),map_frame_id_,odom_frame_id_);
    tf_broadcaster_.sendTransform(stf_odom_map);
    
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
}

void ICPSlamNode::publishMap(ros::Time timestamp)
{
  // TODO: publish the occupancy grid map
  auto map = mapper_.getMapCopy();

  // cout<<"5555555555555555555654646485844444444444444444444444444444444444444444444444444444444444444444"<<endl;
  // cout<<map.size()<<endl;
  // cout<<sizeof(int8_t)<<endl;
  // cout<<map.total()<<endl;

  occupancy_grid_.header.stamp = timestamp;
  // TODO: use map frame
  occupancy_grid_.header.frame_id = map_frame_id_;

  memcpy(occupancy_grid_.data.data(), map.data, map.total()*sizeof(int8_t));
  //cout<<"ssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss"<<endl;
  //cout<<map<<endl;
  map_publisher_.publish(occupancy_grid_);

  cv::imwrite("/tmp/map1.png", map);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "icp_slam_node");
  ICPSlamNode icp_slam_node;
  ros::spin();
  return 0;
}
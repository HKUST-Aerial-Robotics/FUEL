#include "visualization_msgs/Marker.h"
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <vector>

using namespace std;
ros::Publisher cloud_pub;
Eigen::Vector3d map_origin_;
ros::Time last_cloud_time_;

void inflatePoint(const Eigen::Vector3d& pt, int step, vector<Eigen::Vector3d>& pts) {
  const double res = 0.1;
  int num = 0;
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3d(pt(0) + x * res, pt(1) + y * res, pt(2) + z * res);
      }
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  auto tn = ros::Time::now();
  if ((tn - last_cloud_time_).toSec() < 5) {
    return;
  }
  last_cloud_time_ = tn;

  pcl::PointCloud<pcl::PointXYZ> pts;
  pcl::PointCloud<pcl::PointXYZ> pts2;
  pcl::fromROSMsg(*msg, pts);
  vector<Eigen::Vector3d> inf_pts(27);

  for (int i = 0; i < pts.points.size(); ++i) {
    Eigen::Vector3d pt;
    pt(0) = pts[i].x;
    pt(1) = pts[i].y;
    pt(2) = pts[i].z;
    for (int i = 0; i < 3; ++i)
      pt(i) = floor((pt(i) - map_origin_(i)) * 10);
    for (int i = 0; i < 3; ++i)
      pt(i) = (pt(i) + 0.5) * 0.1 + map_origin_(i);
    inflatePoint(pt, 1, inf_pts);
    for (auto pi : inf_pts) {
      pcl::PointXYZ pj;
      pj.x = pi(0);
      pj.y = pi(1);
      pj.z = pi(2);
      pts2.points.push_back(pj);
    }
  }
  pts2.width = pts2.points.size();
  pts2.height = 1;
  pts2.is_dense = true;
  pts2.header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud;
  pcl::toROSMsg(pts2, cloud);
  cloud_pub.publish(cloud);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "process_msg");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber cloud_sub = nh.subscribe("/map_generator/global_cloud", 10, cloudCallback);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/process_msg/global_cloud", 10);
  map_origin_ << -20, -10, -1;

  ros::Duration(1.0).sleep();

  ROS_WARN("[process_msg]: ready.");

  ros::spin();

  return 0;
}
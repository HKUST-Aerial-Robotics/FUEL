#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

ros::Publisher pub_cloud, pub_pose;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;

ros::Timer local_sensing_timer, pose_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

nav_msgs::Odometry odom_;
Eigen::Matrix4d sensor2body, sensor2world;

double sensing_horizon, sensing_rate, estimation_rate;
double x_size, y_size, z_size;
double gl_xl, gl_yl, gl_zl;
double resolution, inv_resolution;
int GLX_SIZE, GLY_SIZE, GLZ_SIZE;

ros::Time last_odom_stamp = ros::TIME_MAX;

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index)
{
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt)
{
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
};

void rcvOdometryCallbck(const nav_msgs::Odometry& odom)
{
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  odom_ = odom;

  Matrix4d body2world = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond pose;
  pose.x() = odom.pose.pose.orientation.x;
  pose.y() = odom.pose.pose.orientation.y;
  pose.z() = odom.pose.pose.orientation.z;
  pose.w() = odom.pose.pose.orientation.w;
  body2world.block<3, 3>(0, 0) = pose.toRotationMatrix();
  body2world(0, 3) = odom.pose.pose.position.x;
  body2world(1, 3) = odom.pose.pose.position.y;
  body2world(2, 3) = odom.pose.pose.position.z;

  // convert to cam pose
  sensor2world = body2world * sensor2body;
}

pcl::PointCloud<pcl::PointXYZ> cloud_all_map, local_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map)
{
  if (has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(cloud_all_map);

  _kdtreeLocalMap.setInputCloud(cloud_all_map.makeShared());

  has_global_map = true;
}

void renderSensedPoints(const ros::TimerEvent& event)
{
  if (!has_global_map || !has_odom)
    return;

  Eigen::Quaterniond q;
  q.x() = odom_.pose.pose.orientation.x;
  q.y() = odom_.pose.pose.orientation.y;
  q.z() = odom_.pose.pose.orientation.z;
  q.w() = odom_.pose.pose.orientation.w;

  Eigen::Vector3d pos;
  pos << odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z;

  Eigen::Matrix3d rot;
  rot = q;
  Eigen::Vector3d yaw_vec = rot.col(0);

  local_map.points.clear();
  pcl::PointXYZ searchPoint(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
      auto pt = cloud_all_map.points[pointIdxRadiusSearch[i]];
      Eigen::Vector3d pt3;
      pt3[0] = pt.x;
      pt3[1] = pt.y;
      pt3[2] = pt.z;
      auto dir = pt3 - pos;

      if (fabs(dir[2]) > dir.head<2>().norm() * tan(M_PI / 6.0))
        continue;

      if (dir.dot(yaw_vec) < 0)
        continue;

      local_map.points.push_back(pt);
    }
    local_map.width = local_map.points.size();
    local_map.height = 1;
    local_map.is_dense = true;

    pcl::toROSMsg(local_map, local_map_pcd);
    local_map_pcd.header = odom_.header;
    pub_cloud.publish(local_map_pcd);
  }
}

void pubSensorPose(const ros::TimerEvent& e)
{
  Eigen::Quaterniond q;
  q = sensor2world.block<3, 3>(0, 0);

  geometry_msgs::PoseStamped sensor_pose;
  sensor_pose.header = odom_.header;
  sensor_pose.header.frame_id = "/map";
  sensor_pose.pose.position.x = sensor2world(0, 3);
  sensor_pose.pose.position.y = sensor2world(1, 3);
  sensor_pose.pose.position.z = sensor2world(2, 3);
  sensor_pose.pose.orientation.w = q.w();
  sensor_pose.pose.orientation.x = q.x();
  sensor_pose.pose.orientation.y = q.y();
  sensor_pose.pose.orientation.z = q.z();
  pub_pose.publish(sensor_pose);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);

  nh.getParam("map/x_size", x_size);
  nh.getParam("map/y_size", y_size);
  nh.getParam("map/z_size", z_size);

  // subscribe point cloud
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // publisher depth image and color image
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/cloud", 10);
  pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pcl_render_node/sensor_pose", 10);
  double sensing_duration = 1.0 / sensing_rate;
  double estimate_duration = 1.0 / estimation_rate;
  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);
  pose_timer = nh.createTimer(ros::Duration(estimate_duration), pubSensorPose);

  inv_resolution = 1.0 / resolution;
  gl_xl = -x_size / 2.0;
  gl_yl = -y_size / 2.0;
  gl_zl = 0.0;
  GLX_SIZE = (int)(x_size * inv_resolution);
  GLY_SIZE = (int)(y_size * inv_resolution);
  GLZ_SIZE = (int)(z_size * inv_resolution);

  sensor2body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}

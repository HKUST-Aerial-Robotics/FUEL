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
#include <pcl/common/common.h>
#include <vector>
// #include <plan_env/raycast.h>
#include <cmath>

using namespace std;
using namespace Eigen;

struct polar3D
{
 int theta;
 int fi;
 float r;
};

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

double sensing_horizon, sensing_rate, estimation_rate, polar_resolution, yaw_fov, vertical_fov, min_raylength, downsample_res;
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
vector<int> pointIdxRadiusSearch2;
vector<float> pointRadiusSquaredDistance2;

void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map)
{
  if (has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  _voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(cloud_all_map);

  _kdtreeLocalMap.setInputCloud(cloud_all_map.makeShared());

  has_global_map = true;
}

void euc2polar(Eigen::Vector3d& euc_pt, float length, polar3D* polar_pt)
{
  // trans from euclidean coordinate to poalr coordinate
  // theta_angle
  // ROS_INFO("Euc point is %f,%f,%f ",euc_pt[0],euc_pt[1],euc_pt[2]);
  polar_pt->theta = round((atan2(euc_pt[1],euc_pt[0])) / M_PI *180.0 / polar_resolution);
  // ROS_INFO("Theta is %d ",polar_pt->theta);
  // fi_angle
  polar_pt->fi = round((atan2(euc_pt[2],euc_pt.head<2>().norm()) / M_PI *180.0/polar_resolution));
  // ROS_INFO("fi is %d ",polar_pt->fi);
  polar_pt->r = length;

}

// class RayCaster;
// unique_ptr<RayCaster> caster_;

int sense_count = 0;
double duration1 = 0.0;
double duration2 = 0.0;
double duration3 = 0.0;
double duration4 = 0.0;
double duration_pub = 0.0;

void renderSensedPoints(const ros::TimerEvent& event)
{

  ros::Time t1 = ros::Time::now();

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
  rot = q.toRotationMatrix();
  Eigen::Vector3d yaw_x;
  yaw_x << 1,0,0;
  Eigen::Vector3d yaw_vec = rot*yaw_x;
  yaw_vec(2) = 0;

  local_map.points.clear();

  Eigen::Vector3d searchpoint_pos;
  Eigen::Vector3d sensorrange_vec;
  sensorrange_vec << 0.5*sensing_horizon, 0, 0;
  searchpoint_pos = pos + rot*sensorrange_vec;

  pcl::PointXYZ searchPoint(searchpoint_pos(0), searchpoint_pos(1), searchpoint_pos(2));
    // pcl::PointXYZ searchPoint(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);

  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  //polar coordinate matrix
  int polar_width = ceil(yaw_fov/polar_resolution);
  int polar_height = ceil(vertical_fov/polar_resolution);
  Eigen::MatrixXf polar_matrix = Eigen::MatrixXf::Zero(polar_width,polar_height);
  Eigen::MatrixXi polarindex_matrix = Eigen::MatrixXi::Zero(polar_width,polar_height);
  int original_pointcount = 0;
  int pointcount = 0;
  int changepointcount = 0;

  sense_count++;
  ros::Time t2 = ros::Time::now();
  duration1 = (duration1 + (t2-t1).toSec());

  if (_kdtreeLocalMap.radiusSearch(searchPoint, 0.5*sensing_horizon, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {

    ros::Time t3 = ros::Time::now();
    duration2 = (duration2 + (t3-t2).toSec());
    duration4 = 0.0;
    double top_dura= 0.0;
    double down_dura= 0.0;

    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {

      ros::Time t5 = ros::Time::now();

      auto pt = cloud_all_map.points[pointIdxRadiusSearch[i]];
      Eigen::Vector3d pt3;
      pt3[0] = pt.x;
      pt3[1] = pt.y;
      pt3[2] = pt.z;
      auto dir = pt3 - pos;

      //this is a simple 2D realize
      if (fabs(dir[2]) > dir.head<2>().norm() * tan(M_PI * (vertical_fov/2.0)/180.0))
        continue;

      if (dir.dot(yaw_vec) < 0)
        continue;

      Vector3d dir_xy = dir;
      dir_xy(2) = 0;
      // ROS_INFO("YAWVEC = %f,%f, dir = %f,%f",yaw_vec(0),yaw_vec(1),dir(0),dir(1));
      if (acos(dir.dot(yaw_vec) / (dir.norm()*yaw_vec.norm())) > (M_PI * (yaw_fov/2.0)/180.0))//add yaw fov  0.1944
        continue;

      //dead zone 
      if (dir.norm() < min_raylength*min_raylength)
        continue;
      
      // ROS_INFO("After min ray length");

      original_pointcount++;

      //use polar to filter the right pointclouds
      polar3D polar_pt;
      Eigen::Vector3d dir_vec;
      dir_vec = pt3 - pos;
      
      //trans coordinate to lidar coordinate
      dir_vec = rot.transpose() * dir_vec;

      euc2polar(dir_vec, dir_vec.norm(), &polar_pt);
      // ROS_INFO("dir_vec = %f,%f,%f, polar = %d,%d",dir(0),dir(1),dir(2), polar_pt.theta,polar_pt.fi);
      int cen_theta_index = polar_pt.theta + round(0.5 * polar_width);
      int cen_fi_index = polar_pt.fi + round(0.5 * polar_height);

      int half_cover_angle = ceil((asin(downsample_res/dir_vec.norm()) / (M_PI* polar_resolution/180.0)));
      // ROS_INFO("half cover angle = %d",half_cover_angle);
      // int half_cover_angle = 0;

      ros::Time t6 = ros::Time::now();
      duration4 = (duration4 + (t6-t5).toSec());

      int filter_count =0;
      // double top_dura= 0.0;
      // double down_dura= 0.0;
      for(int theta_index_o=cen_theta_index-half_cover_angle;theta_index_o<=cen_theta_index+half_cover_angle;theta_index_o++)
      {
          for(int fi_index_o=cen_fi_index-half_cover_angle;fi_index_o<=cen_fi_index+half_cover_angle;fi_index_o++)
          {

      // ros::Time t8 = ros::Time::now();

      int theta_index = theta_index_o;
      int fi_index = fi_index_o;
      if(theta_index > (polar_width-1))
      {
        continue;
        theta_index = (polar_width-1);
      }else if(theta_index < 0)
      {
        continue;
        theta_index = 0;
      }

      if(fi_index > (polar_height-1))
      {
        continue;
        fi_index = (polar_height-1);
      }else if(fi_index < 0)
      {
        continue;
        fi_index = 0;
      }

      // theta_index > (polar_width-1) ? (polar_width-1):theta_index;
      // theta_index < 0 ? 0:theta_index;
      // fi_index > (polar_height-1) ? (polar_height-1):fi_index;
      // fi_index < 0 ? 0:fi_index;

      // filter_count ++;
      // ros::Time t9 = ros::Time::now();
      // top_dura = (top_dura + (t9-t8).toSec());

      // ROS_INFO("Get into polar, (%d,%d) point is %f, current point r = %f, polarindex = %d",theta_index,fi_index,polar_matrix(theta_index,fi_index),polar_pt.r, polarindex_matrix(theta_index,fi_index));
      if (polar_matrix(theta_index,fi_index) == 0.0)//fabs(polar_matrix(theta_index,fi_index) - 0.0) < 0.1
      {
        // ROS_INFO("Dealing point %d ",(int)i);

        polar_matrix(theta_index,fi_index) = polar_pt.r;
        polarindex_matrix(theta_index,fi_index) = pointcount;
        local_map.points.push_back(pt);
        pointcount++;

        // ROS_INFO("Add point %d: polar %d, %d, %lf",i, theta_index, fi_index,polar_pt.r);
      }
      else if(polar_matrix(theta_index,fi_index) > polar_pt.r)
      {
        polar_matrix(theta_index,fi_index) = polar_pt.r;
        // ROS_INFO("Change point is %d",polarindex_matrix(theta_index,fi_index));
        local_map.points[polarindex_matrix(theta_index,fi_index)] = pt;
        changepointcount++;
        // polarindex_matrix(theta_index,fi_index) = i+1;

        // ROS_INFO("Change point %d: polar %d, %d, %f",(int)i, theta_index, fi_index,polar_pt.r);
      }
      // local_map.points.push_back(pt);

      // ros::Time t10 = ros::Time::now();
      // down_dura = (down_dura + (t10-t9).toSec());
          }
      }
      
    }
    // ROS_INFO("top=%f, down = %f",top_dura,down_dura);

    ros::Time t4 = ros::Time::now();
    // duration3 = (duration3 + (t4-t3).toSec())/ sense_count;
    duration3 =  (t4-t3).toSec()+duration3;

    ROS_INFO("GET OUT OF LOOP, pointcount = %d, origin pointcount = %d, change point = %d",pointcount,original_pointcount,changepointcount);
    local_map.width = local_map.points.size();
    local_map.height = 1;
    local_map.is_dense = true;

    pcl::toROSMsg(local_map, local_map_pcd);
    local_map_pcd.header = odom_.header;
    pub_cloud.publish(local_map_pcd);

    ros::Time t7 = ros::Time::now();
    duration_pub = (duration_pub + (t7-t4).toSec());

  }
  ros::Time t_total = ros::Time::now();
  // std::cout << "ONE FOV GENERATE:"<<  (t_total-t1).toSec()<<std::endl;;
  ROS_INFO("Time statics: %f,%f,%f,%f,%f total_time = %f",duration1/sense_count,duration2/sense_count,duration3/sense_count,duration4/pointIdxRadiusSearch.size(),duration_pub/sense_count,(t_total-t1).toSec());

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
  nh.getParam("polar_resolution", polar_resolution);
  nh.getParam("yaw_fov", yaw_fov);
  nh.getParam("vertical_fov", vertical_fov);
  nh.getParam("min_raylength", min_raylength);
  nh.getParam("downsample_res", downsample_res);

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

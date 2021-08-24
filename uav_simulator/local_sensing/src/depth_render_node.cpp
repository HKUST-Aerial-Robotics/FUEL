#include <fstream>
#include <iostream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <pcl_conversions/pcl_conversions.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <Eigen/Eigen>

#include <backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

//#include <cloud_banchmark/cloud_banchmarkConfig.h>
using namespace cv;
using namespace std;
using namespace Eigen;

cv::Mat depth_mat;

// camera param
int width, height;
double fx, fy, cx, cy;
double sensing_horizon, sensing_rate, estimation_rate;

ros::Publisher pub_depth;
ros::Publisher pub_pose;
ros::Subscriber odom_sub;
ros::Subscriber global_map_sub;
ros::Timer local_sensing_timer, estimation_timer;

bool has_global_map(false);
bool has_odom(false);

Matrix4d cam02body;
Matrix4d cam2world;
Eigen::Quaterniond cam2world_quat;
nav_msgs::Odometry odom_;

ros::Time last_odom_stamp = ros::TIME_MAX;
Eigen::Vector3d last_pose_world;
pcl::PointCloud<pcl::PointXYZ> cloudIn;

void odometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  odom_ = odom;
  Matrix4d Pose_receive = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;
  request_pose.x() = odom.pose.pose.orientation.x;
  request_pose.y() = odom.pose.pose.orientation.y;
  request_pose.z() = odom.pose.pose.orientation.z;
  request_pose.w() = odom.pose.pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Matrix4d body_pose = Pose_receive;
  // convert to cam pose
  cam2world = body_pose * cam02body;
  cam2world_quat = cam2world.block<3, 3>(0, 0);
  last_odom_stamp = odom.header.stamp;

  last_pose_world(0) = odom.pose.pose.position.x;
  last_pose_world(1) = odom.pose.pose.position.y;
  last_pose_world(2) = odom.pose.pose.position.z;
}

void pubCameraPose(const ros::TimerEvent& event) {
  // cout<<"pub cam pose"
  geometry_msgs::PoseStamped camera_pose;
  camera_pose.header = odom_.header;
  camera_pose.header.frame_id = "/map";
  camera_pose.pose.position.x = cam2world(0, 3);
  camera_pose.pose.position.y = cam2world(1, 3);
  camera_pose.pose.position.z = cam2world(2, 3);
  camera_pose.pose.orientation.w = cam2world_quat.w();
  camera_pose.pose.orientation.x = cam2world_quat.x();
  camera_pose.pose.orientation.y = cam2world_quat.y();
  camera_pose.pose.orientation.z = cam2world_quat.z();
  pub_pose.publish(camera_pose);
}

void pointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map) return;

  ROS_WARN("Global Pointcloud received..");
  // load global map
  // transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);
  printf("global map has points: %d.\n", cloudIn.points.size());
  has_global_map = true;
}

void renderDepth() {
  double this_time = ros::Time::now().toSec();
  Matrix4d cam_pose = cam2world.inverse();

  // depth_mat = cv::Mat::zeros();
  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);

  Eigen::Matrix4d Tcw = cam2world.inverse();
  Eigen::Matrix3d Rcw = Tcw.block<3, 3>(0, 0);
  Eigen::Vector3d tcw = Tcw.block<3, 1>(0, 3);

  auto t1 = ros::Time::now();

  Eigen::Vector3d pos = cam2world.block<3, 1>(0, 3);
  for (auto pt : cloudIn.points) {
    Eigen::Vector3d pw(pt.x, pt.y, pt.z);
    if ((pos - pw).norm() > 5.0) continue;

    Eigen::Vector3d pc = Rcw * pw + tcw;

    if (pc[2] <= 0.0) continue;

    // std::cout << "pc: " << pc.transpose() << std::endl;

    float projected_x, projected_y;
    projected_x = pc[0] / pc[2] * fx + cx;
    projected_y = pc[1] / pc[2] * fy + cy;
    if (projected_x < 0 || projected_x >= width || projected_y < 0 || projected_y >= height)
      continue;

    // std::cout << "(u,v): " << projected_x << ", " << projected_y << endl;
    float dist = pc[2];
    int r = 0.0573 * fx / dist + 0.5;
    // std::cout << "r: " << r << std::endl;
    int min_x = max(int(projected_x - r), 0);
    int max_x = min(int(projected_x + r), width - 1);
    int min_y = max(int(projected_y - r), 0);
    int max_y = min(int(projected_y + r), height - 1);

    for (int to_x = min_x; to_x <= max_x; to_x++)
      for (int to_y = min_y; to_y <= max_y; to_y++) {
        // std::cout << "(u',v'): " << to_x << ", " << to_y << std::endl;
        float value = depth_mat.at<float>(to_y, to_x);
        if (value < 1e-3) {
          depth_mat.at<float>(to_y, to_x) = dist;
        } else {
          depth_mat.at<float>(to_y, to_x) = min(value, dist);
        }
      }
  }

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = last_odom_stamp;
  out_msg.header.frame_id = "world";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  pub_depth.publish(out_msg.toImageMsg());
}

void renderSensedPoints(const ros::TimerEvent& event) {
  if (!has_global_map) return;
  renderDepth();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("cam_width", width);
  nh.getParam("cam_height", height);
  nh.getParam("cam_fx", fx);
  nh.getParam("cam_fy", fy);
  nh.getParam("cam_cx", cx);
  nh.getParam("cam_cy", cy);
  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);
  cam02body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  // init cam2world transformation
  cam2world = Matrix4d::Identity();
  // subscribe point cloud
  global_map_sub = nh.subscribe("global_map", 1, pointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, odometryCallbck);

  // publisher depth image and color image
  pub_depth = nh.advertise<sensor_msgs::Image>("/pcl_render_node/depth", 1000);
  pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pcl_render_node/sensor_pose", 1000);

  double sensing_duration = 1.0 / sensing_rate;
  double estimate_duration = 1.0 / estimation_rate;

  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);
  estimation_timer = nh.createTimer(ros::Duration(estimate_duration), pubCameraPose);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}
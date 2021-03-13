/* ----------------------------------------------------------------------------
 * used for benchmark
 * -------------------------------------------------------------------------- */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <iostream>
#include <quadrotor_msgs/PositionCommand.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

ros::Publisher traj_pub, traj_pub2, yaw_pub, cloud_pub, ewok_pub;

ros::Time start_time, end_time, last_time;
Eigen::Vector3d last_pos, last_acc;
vector<Eigen::Vector3d> traj, vel, yaw1, yaw2;
vector<double> yaw;
vector<Eigen::Vector3d> cam1, cam2;
pcl::PointCloud<pcl::PointXYZ>::Ptr pts;

// calculate flight time and distance
double distance1;

void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                     double line_width, const Eigen::Vector4d& color, int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  yaw_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  yaw_pub.publish(mk);

  ros::Duration(0.001).sleep();
}

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
  ROS_INFO_ONCE("start");
  Eigen::Vector3d pt, v;
  pt(0) = msg->position.x;
  pt(1) = msg->position.y;
  pt(2) = msg->position.z;
  v(0) = msg->velocity.x;
  v(1) = msg->velocity.y;
  v(2) = msg->velocity.z;
  auto cur_time = ros::Time::now();

  // if (pt(0) > 5 && pt(1) < 3) return;

  if (traj.size() > 0) {
    distance1 += (pt - traj.back()).norm();
  }

  traj.push_back(pt);
  vel.push_back(v);
  displayTrajWithColor(traj, 0.1, Eigen::Vector4d(1, 0, 0, 1), 0);

  double phi = msg->yaw;
  yaw.push_back(phi);
  yaw1.clear();
  yaw2.clear();
  for (int k = 0; k < 4; ++k) {
    int idx = yaw.size() - 1 - 30 * k;
    if (idx < 0) continue;
    double phi_k = yaw[idx];
    Eigen::Vector3d pt_k = traj[idx];
    Eigen::Matrix3d Rwb;
    Rwb << cos(phi_k), -sin(phi_k), 0, sin(phi_k), cos(phi_k), 0, 0, 0, 1;
    for (int i = 0; i < cam1.size(); ++i) {
      auto p1 = Rwb * cam1[i] + pt_k;
      auto p2 = Rwb * cam2[i] + pt_k;
      yaw1.push_back(p1);
      yaw2.push_back(p2);
    }
  }
  // displayLineList(yaw1, yaw2, 0.02, Eigen::Vector4d(0, 0, 0, 1), 0);

  // std::cout << v(0) << "," << v(1) << "," << v(2) << ",";
  if (v.norm() < 1e-3) {
    ROS_INFO("end, distance: %lf", distance1);
  }
}

bool endtraj = false;
void trajCallback(const visualization_msgs::MarkerConstPtr& msg) {
  // if (msg->id == 500) return;
  // std::cout << msg->id << std::endl;
  if (msg->id != 399) return;

  visualization_msgs::Marker mk;
  mk = *msg;
  mk.color.a = 0.3;
  mk.scale.x = 0.1;
  mk.scale.y = 0.1;
  mk.scale.z = 0.1;
  mk.id = 1;

  // if(mk.points.size()>0 && mk.points[0].y < 3 && mk.points[0].x >5) return;

  // int end = -1;
  // for (int i = 0; i < mk.points.size(); ++i) {
  //   auto pt = mk.points[i];
  //   if (pt.x > 5 && pt.y < 3) {
  //     end = i;
  //     break;

  //   }
  // }
  // if (end > 0) mk.points.erase(mk.points.begin() + end, mk.points.end());

  traj_pub2.publish(mk);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  static int msg_num = 0;
  if (++msg_num % 10 != 0) return;

  pcl::PointCloud<pcl::PointXYZ> pts2, pts3;
  pcl::fromROSMsg(*msg, pts2);

  // Filter unwanted points
  for (int i = 0; i < pts2.points.size(); ++i) {
    // if (pts2[i].z > 0.0 && pts2[i].z < 0.1 || pts2[i].z > 3.5 || pts2[i].y < -3.5)
    // continue;
    // if (pts2[i].z < 0.0) continue;
    // if (pts2[i].z > 1.5 && pts2[i].x < 5.0) continue;
    // if (pts2[i].y < 0 && pts2[i].y > -1 && pts2[i].z > 0.1 && pts2[i].x > 7.8)
    // continue;
    // if (pts2[i].y > 2.8) continue;
    pts->push_back(pts2[i]);
  }

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(pts);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(pts3);

  pts->points = pts3.points;
  pts->width = pts->points.size();
  pts->height = 1;
  pts->is_dense = true;
  pts->header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud;
  pcl::toROSMsg(*pts, cloud);
  cloud_pub.publish(cloud);
}

void ewokCallback(const visualization_msgs::MarkerArrayConstPtr& msg) {
  auto marker = msg->markers[0];
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  ewok_pub.publish(marker);
}

void travelCallback(const visualization_msgs::MarkerPtr& msg) {
  if (msg->id == 5) {
    msg->color.g = 0.8;
  }
  traj_pub2.publish(*msg);
}

int main(int argc, char** argv) {
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "faster");
  ros::NodeHandle nh("~");

  ros::Subscriber cmd_sub = nh.subscribe("/position_cmd", 10, cmdCallback);
  // ros::Subscriber cloud_sub = nh.subscribe("/sdf_map/occupancy", 10, cloudCallback);
  ros::Subscriber cloud_sub = nh.subscribe("/sdf_map/occupancy_local", 10, cloudCallback);
  ros::Subscriber traj_sub = nh.subscribe("/planning_vis/trajectory", 10, trajCallback);
  ros::Subscriber ewok_sub = nh.subscribe("/firefly/optimal_trajectory", 10, ewokCallback);
  // ros::Subscriber travel_traj_sub = nh.subscribe("/planning/travel_traj", 10,
  // travelCallback);

  traj_pub = nh.advertise<visualization_msgs::Marker>("/process_msg/execute_traj", 10);
  yaw_pub = nh.advertise<visualization_msgs::Marker>("/process_msg/execute_yaw", 10);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/process_msg/global_cloud", 10);
  ewok_pub = nh.advertise<visualization_msgs::Marker>("/process_msg/ewok", 10);
  // traj_pub2 = nh.advertise<visualization_msgs::Marker>("/process_msg/plan_traj", 10);
  traj_pub2 = nh.advertise<visualization_msgs::Marker>("/planning/travel_traj", 10);

  last_pos.setZero();
  pts.reset(new pcl::PointCloud<pcl::PointXYZ>());

  // draw camera FOV in body frame
  /* camera FOV vertice */
  const double vert_ang = 0.56125;
  const double hor_ang = 0.68901;
  const double cam_scale = 0.5;
  double hor = cam_scale * tan(hor_ang);
  double vert = cam_scale * tan(vert_ang);
  Eigen::Vector3d origin(0, 0, 0);
  Eigen::Vector3d left_up(cam_scale, hor, vert);
  Eigen::Vector3d left_down(cam_scale, hor, -vert);
  Eigen::Vector3d right_up(cam_scale, -hor, vert);
  Eigen::Vector3d right_down(cam_scale, -hor, -vert);

  /* draw line between vertice */
  cam1.push_back(origin);
  cam2.push_back(left_up);
  cam1.push_back(origin);
  cam2.push_back(left_down);
  cam1.push_back(origin);
  cam2.push_back(right_up);
  cam1.push_back(origin);
  cam2.push_back(right_down);

  cam1.push_back(left_up);
  cam2.push_back(right_up);
  cam1.push_back(right_up);
  cam2.push_back(right_down);
  cam1.push_back(right_down);
  cam2.push_back(left_down);
  cam1.push_back(left_down);
  cam2.push_back(left_up);

  distance1 = 0;

  ros::spin();  // spin the normal queue

  return 0;
}
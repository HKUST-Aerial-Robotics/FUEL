
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

ros::Publisher marker1_pub_;

vector<Eigen::Vector3d> cam1, cam2;
Eigen::Vector3d last_cmd_pos_;
double last_yaw_;

double alpha_;
visualization_msgs::Marker view_mk_;

void drawLines(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
               const double& line_width, const Eigen::Vector4d& color, const string& ns, const int& id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  mk.ns = ns;
  marker1_pub_.publish(mk);

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
  marker1_pub_.publish(mk);

  ros::Duration(0.001).sleep();
}

void drawSpheres(const vector<Eigen::Vector3d>& points, const double& resolution,
                 const Eigen::Vector4d& color, const string& ns, const int& id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  mk.ns = ns;
  marker1_pub_.publish(mk);

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
  for (int i = 0; i < int(points.size()); i++) {
    pt.x = points[i](0);
    pt.y = points[i](1);
    pt.z = points[i](2);
    mk.points.push_back(pt);
  }
  marker1_pub_.publish(mk);
  ros::Duration(0.001).sleep();
}

void calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]
  double round_last = last_yaw;
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;
  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

void fovCallback(const visualization_msgs::MarkerConstPtr& msg) {
  if (msg->points.size() == 0) return;

  // Calculate the position and yaw
  Eigen::Vector3d p0(msg->points[0].x, msg->points[0].y, msg->points[0].z);  // Camera origin
  last_cmd_pos_ = p0;

  Eigen::Vector3d p1(msg->points[1].x, msg->points[1].y, msg->points[1].z);  // Left up
  Eigen::Vector3d p5(msg->points[5].x, msg->points[5].y, msg->points[5].z);  // Right up
  Eigen::Vector3d dir = p1 - p0 + p5 - p0;
  double tmp_yaw = atan2(dir[1], dir[0]);
  calcNextYaw(last_yaw_, tmp_yaw);

  // Do yaw low pass filtering
  double yaw = (1 - alpha_) * tmp_yaw + alpha_ * last_yaw_;
  last_yaw_ = yaw;

  // Draw filtered FOV
  Eigen::Matrix3d Rwb;
  Rwb << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  vector<Eigen::Vector3d> l1, l2;
  for (int i = 0; i < cam1.size(); ++i) {
    l1.push_back(Rwb * cam1[i] + p0);
    l2.push_back(Rwb * cam2[i] + p0);
  }
  drawLines(l1, l2, 0.04, Eigen::Vector4d(0, 0, 0, 1), "fov", 0);
}

void cmdTrajCallback(const visualization_msgs::MarkerConstPtr& msg) {
  if (msg->points.size() == 0) return;

  // Video
  // visualization_msgs::Marker mk;
  // mk = *msg;

  // // Change color
  // mk.color.r = 1;
  // mk.color.g = 0.75;
  // mk.color.b = 0;
  // mk.color.a = 1;

  // // Change namespace
  // mk.id = 0;
  // mk.ns = "execute_traj";
  // marker1_pub_.publish(mk);

  // Benchmark bridge
  visualization_msgs::Marker mk;
  mk = *msg;

  // Change scale
  mk.scale.x = 0.1;
  mk.scale.y = 0.1;
  mk.scale.z = 0.1;

  if (mk.id == 2) {
    mk.color.r = 1;
    mk.color.g = 0;
    mk.color.b = 0;
    mk.ns = "classic";

    for (auto& p : mk.points) {
      // p.x = -p.x;
      p.y = -p.y;
    }
  }
  if (mk.id == 3) {
    mk.color.r = 0;
    mk.color.g = 1;
    mk.color.b = 0;
    mk.ns = "rapid";

    // for (auto& p : mk.points)
    //   p.y = -p.y;
  }
  if (mk.id == 4) {
    mk.color.r = 0;
    mk.color.g = 0;
    mk.color.b = 1;
    mk.ns = "propose";
  }

  // // Benchmark maze
  // visualization_msgs::Marker mk;
  // mk = *msg;

  // // Change scale
  // mk.scale.x = 0.15;
  // mk.scale.y = 0.15;
  // mk.scale.z = 0.15;

  // if (mk.id == 2)
  // {
  //   mk.color.r = 1;
  //   mk.color.g = 0;
  //   mk.color.b = 0;
  //   mk.ns = "classic";
  // }
  // if (mk.id == 3)
  // {
  //   mk.color.r = 0;
  //   mk.color.g = 1;
  //   mk.color.b = 0;
  //   mk.ns = "rapid";

  //   for (auto iter = mk.points.begin(); iter != mk.points.end();)
  //   {
  //     if (iter->x < -2.5 && iter->x > -4 && iter->y < -4 && iter->y > -6)
  //     {
  //       iter = mk.points.erase(iter);
  //       std::cout << "erase" << std::endl;
  //     }
  //     else
  //       ++iter;
  //   }
  // }
  // if (mk.id == 4)
  // {
  //   mk.color.r = 0;
  //   mk.color.g = 0;
  //   mk.color.b = 1;
  //   mk.ns = "propose";
  // }

  // Change namespace
  marker1_pub_.publish(mk);
}

void planTrajCallback(const visualization_msgs::MarkerConstPtr& msg) {
  if (msg->points.size() == 0) return;

  // Offset by difference with current FOV
  visualization_msgs::Marker mk = *msg;
  Eigen::Vector3d p0(mk.points[0].x, mk.points[0].y, mk.points[0].z);
  Eigen::Vector3d diff = last_cmd_pos_ - p0;
  for (auto& p : mk.points) {
    p.x += diff[0];
    p.y += diff[1];
    p.z += diff[2];
  }
  mk.color.a = 0.5;
  mk.ns = "plan_traj";
  mk.id = 0;
  marker1_pub_.publish(mk);

  // Draw next FOV
  Eigen::Vector3d p0_next(view_mk_.points[0].x, view_mk_.points[0].y, view_mk_.points[0].z);
  Eigen::Vector3d p1(view_mk_.points[1].x, view_mk_.points[1].y, view_mk_.points[1].z);  // Left up
  Eigen::Vector3d p5(view_mk_.points[5].x, view_mk_.points[5].y, view_mk_.points[5].z);  // Right up
  Eigen::Vector3d dir = p1 + p5 - 2 * p0_next;
  double next_yaw = atan2(dir[1], dir[0]);
  Eigen::Matrix3d Rwb;
  Rwb << cos(next_yaw), -sin(next_yaw), 0, sin(next_yaw), cos(next_yaw), 0, 0, 0, 1;

  auto p_msg_end = mk.points.back();
  Eigen::Vector3d p_end(p_msg_end.x, p_msg_end.y, p_msg_end.z);

  vector<Eigen::Vector3d> l1, l2;
  for (int i = 0; i < cam1.size(); ++i) {
    l1.push_back(Rwb * cam1[i] + p_end);
    l2.push_back(Rwb * cam2[i] + p_end);
  }
  drawLines(l1, l2, 0.04, Eigen::Vector4d(1, 0, 0, 1), "plan_traj", 1);
}

void viewCallback(const visualization_msgs::MarkerConstPtr& msg) {
  if (msg->ns == "global_tour" && msg->points.size() == 0) {
    visualization_msgs::Marker mk = *msg;
    mk.ns = "plan_traj";
    marker1_pub_.publish(mk);
    mk.ns = "next_fov";
    marker1_pub_.publish(mk);
    return;
  }

  if (msg->ns != "refined_view" || msg->points.size() == 0) return;
  view_mk_ = *msg;
  view_mk_.points.erase(view_mk_.points.begin() + 16, view_mk_.points.end());
}

void nbvpCallback(const visualization_msgs::MarkerConstPtr& msg) {
  visualization_msgs::Marker mk = *msg;
  mk.scale.x = 0.1;
  mk.scale.y = 0.1;
  mk.scale.z = 0.1;

  mk.color.r = 1;
  mk.color.g = 0;
  mk.color.b = 1;
  int size = mk.points.size();
  mk.ns = "nbvp";

  // Bridge
  for (auto& p : mk.points) {
    double tmpx = p.x;
    double tmpy = p.y;
    p.x = -tmpy;
    p.y = -tmpx;
  }

  // Maze
  auto& pts = mk.points;
  // mk.points.erase(pts.begin() + size / 3, pts.end());

  marker1_pub_.publish(mk);
  return;
}

int main(int argc, char** argv) {
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "process_msg2");
  ros::NodeHandle nh("~");

  ros::Subscriber cmd_sub = nh.subscribe("/planning/position_cmd_vis", 10, fovCallback);
  ros::Subscriber cmd_traj_sub = nh.subscribe("/planning/travel_traj", 10, cmdTrajCallback);
  ros::Subscriber plan_traj_sub = nh.subscribe("/planning_vis/trajectory", 10, planTrajCallback);
  ros::Subscriber view_sub = nh.subscribe("/planning_vis/viewpoints", 10, viewCallback);
  ros::Subscriber nbvp_sub = nh.subscribe("/firefly/visualization_marker", 10, nbvpCallback);

  marker1_pub_ = nh.advertise<visualization_msgs::Marker>("/process_msg/marker1", 10);

  nh.param("process_msg/alpha", alpha_, 0.9);

  // Camera FOV vertice
  const double vert_ang = 0.56125;
  const double hor_ang = 0.68901;
  const double cam_scale = 0.8;
  double hor = cam_scale * tan(hor_ang);
  double vert = cam_scale * tan(vert_ang);
  Eigen::Vector3d origin(0, 0, 0);
  Eigen::Vector3d left_up(cam_scale, hor, vert);
  Eigen::Vector3d left_down(cam_scale, hor, -vert);
  Eigen::Vector3d right_up(cam_scale, -hor, vert);
  Eigen::Vector3d right_down(cam_scale, -hor, -vert);
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

  ros::spin();

  return 0;
}
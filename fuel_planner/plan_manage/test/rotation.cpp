#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotation");
  ros::NodeHandle node("~");

  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("/rotation/odom", 10);
  ros::Publisher mark_pub = node.advertise<visualization_msgs::Marker>("/rotation/point", 10);
  ros::Duration(1.0).sleep();

  // write the euler angle represented rotation matrix
  Matrix3d Rx, Ry, Rz;
  const double a = 0.785;

  Rx << 1.0, 0.0, 0.0, 0.0, cos(a), -sin(a), 0.0, sin(a), cos(a);

  Ry << cos(a), 0.0, sin(a), 0.0, 1.0, 0.0, -sin(a), 0.0, cos(a);

  Rz << cos(a), -sin(a), 0.0, sin(a), cos(a), 0.0, 0.0, 0.0, 1.0;

  Matrix3d R1 = Rz;
  Matrix3d R2 = Rz * Ry;
  Matrix3d R3 = Rz * Ry * Rx;

  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp = ros::Time::now();

  // pub R
  Quaterniond q1(R1);
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = 1.0;
  odom.pose.pose.orientation.w = q1.w();
  odom.pose.pose.orientation.x = q1.x();
  odom.pose.pose.orientation.y = q1.y();
  odom.pose.pose.orientation.z = q1.z();
  odom_pub.publish(odom);

  Quaterniond q2(R2);
  odom.pose.pose.position.x = 2.0;
  odom.pose.pose.position.y = 2.0;
  odom.pose.pose.orientation.w = q2.w();
  odom.pose.pose.orientation.x = q2.x();
  odom.pose.pose.orientation.y = q2.y();
  odom.pose.pose.orientation.z = q2.z();
  odom_pub.publish(odom);

  Quaterniond q3(R3);
  odom.pose.pose.position.x = 3.0;
  odom.pose.pose.position.y = 3.0;
  odom.pose.pose.orientation.w = q3.w();
  odom.pose.pose.orientation.x = q3.x();
  odom.pose.pose.orientation.y = q3.y();
  odom.pose.pose.orientation.z = q3.z();
  odom_pub.publish(odom);

  // test the transformed points
  Vector3d p1, p2, p3, p4;
  Vector3d p0(1, 0, 0);
  p1 = R1 * p0;
  p2 = R2 * p0;
  p3 = Ry * p0;
  p4 = Rz * Ry * p0;

  visualization_msgs::Marker m;
  m.header.frame_id = "world";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.id = 0;
  m.color.r = 1;
  m.color.a = 1;

  geometry_msgs::Point p;
  p.x = p1(0);
  p.y = p1(1);
  p.z = p1(2);
  m.points.push_back(p);
  p.x = p2(0);
  p.y = p2(1);
  p.z = p2(2);
  m.points.push_back(p);

  p.x = p3(0);
  p.y = p3(1);
  p.z = p3(2);
  m.points.push_back(p);
  p.x = p4(0);
  p.y = p4(1);
  p.z = p4(2);
  m.points.push_back(p);

  mark_pub.publish(m);

  ros::Duration(1.0).sleep();

  return 1;
}
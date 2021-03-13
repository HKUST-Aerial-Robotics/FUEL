#include <iostream>
#include <ros/ros.h>
#include <pcl/io/obj_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "load_obj");
  ros::NodeHandle node("~");

  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/load_obj/cloud", 10);

  pcl::PointCloud<pcl::PointXYZ> cloud;

  // pcl::io::loadOBJFile("/home/boboyu/Downloads/AnyConv.com__truss_bridge.obj", cloud);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/boboyu/Downloads/pp.pcd", cloud);

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "world";

  // Rotate the cloud
  for (int i = 0; i < cloud.points.size(); ++i) {
    auto pt = cloud.points[i];
    pcl::PointXYZ pr;
    pr.x = pt.x;
    pr.y = -pt.z;
    pr.z = pt.y;
    cloud.points[i] = pr;
  }

  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(cloud, cloud2);

  while (ros::ok()) {
    cloud_pub.publish(cloud2);
    ros::Duration(0.2).sleep();
  }

  std::cout << "Cloud published!" << std::endl;

  return 1;
}

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

using namespace std;

bool finish = false;
string file_path;

void cloudCallback(const sensor_msgs::PointCloud2& msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(msg, cloud);
  pcl::io::savePCDFileASCII(file_path + std::string("tmp.pcd"), cloud);

  cout << "map saved." << endl;
  finish = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_recorder");
  ros::NodeHandle node;

  if (argc <= 1) {
    std::cout << "File path not specified" << std::endl;
    return 0;
  }

  file_path = argv[1];

  // ros::Subscriber cloud_sub = node.subscribe("/map_generator/global_cloud",
  // 10, cloudCallback);
  // ros::Subscriber cloud_sub =
  // node.subscribe("/firefly/nbvPlanner/octomap_pcl", 10, cloudCallback);

  // Generate map by clicking
  ros::Subscriber cloud_sub = node.subscribe("/map_generator/click_map", 10, cloudCallback);
  ros::Duration(1.0).sleep();

  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    if (finish) break;
  }

  // cout << "finish record map." << endl;
  ROS_WARN("[Map Recorder]: finish record map.");
  return 0;
}

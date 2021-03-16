#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

bool finish = false;

void cloudCallback(const sensor_msgs::PointCloud2& msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(msg, cloud);
  pcl::io::savePCDFileASCII("/home/boboyu/workspaces/plan_ws/src/uav_simulator/map_generator/resource/test_map.pcd",
                            cloud);
                            
  cout << "map saved." << endl;
  finish = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_recorder");
  ros::NodeHandle node;

  // ros::Subscriber cloud_sub = node.subscribe("/map_generator/global_cloud", 10, cloudCallback);
  ros::Subscriber cloud_sub = node.subscribe("/firefly/nbvPlanner/octomap_pcl", 10, cloudCallback);
  ros::Duration(1.0).sleep();

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    if (finish)
      break;
  }

  // cout << "finish record map." << endl;
  ROS_WARN("[Map Recorder]: finish record map.");
  return 0;
}

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
string file_name;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_recorder");
  ros::NodeHandle node;

  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 10, true);
  file_name = argv[1];

  ros::Duration(1.0).sleep();

  /* load cloud from pcd */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud);
  if (status == -1)
  {
    cout << "can't read file." << endl;
    return -1;
  }

  // // Process map
  // for (int i = 0; i < cloud.points.size(); ++i)
  // {
  //   auto pt = cloud.points[i];
  //   pcl::PointXYZ pr;
  //   pr.x = pt.x;
  //   pr.y = -pt.z;
  //   pr.z = pt.y;
  //   cloud.points[i] = pr;
  // }

  for (double x = -10; x <= 10; x += 0.05)
    for (double y = -10; y <= 10; y += 0.05)
    {
      cloud.push_back(pcl::PointXYZ(x, y, 0));
    }

  // cout << "Publishing map..." << endl;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "world";

  int count = 0;
  while (ros::ok())
  {
    ros::Duration(0.3).sleep();
    cloud_pub.publish(msg);
    ++count;
    if (count > 10)
    {
      // break;
    }
  }

  cout << "finish publish map." << endl;

  return 0;
}
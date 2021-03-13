#include "std_msgs/Empty.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <fstream>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <stdlib.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace std;
using namespace fast_planner;

pcl::PointCloud<pcl::PointXYZ> latest_cloud;

double start_x, start_y, start_z;
double start_vx, start_vy, start_vz;
double goal_x, goal_y, goal_z;

bool have_goal, have_map;

/* ---------- receive planning map ---------- */
void mapCallback(const sensor_msgs::PointCloud2& msg) {
  pcl::fromROSMsg(msg, latest_cloud);

  if ((int)latest_cloud.points.size() == 0) return;

  have_map = true;
  std::cout << "[1]: get map" << std::endl;
}

/* ---------- receive start and goal ---------- */
void sgCallback(const sensor_msgs::PointCloud& msg) {
  if (msg.points.size() != 3) {
    std::cout << "sg num error." << std::endl;
    return;
  } else {
    cout << "get sg msg." << endl;
  }

  start_x = msg.points[0].x, start_y = msg.points[0].y, start_z = msg.points[0].z;
  start_vx = msg.points[1].x, start_vy = msg.points[1].y, start_vz = msg.points[1].z;
  goal_x = msg.points[2].x, goal_y = msg.points[2].y, goal_z = msg.points[2].z;

  have_goal = true;
  std::cout << "[1]: get sg" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "random");
  ros::NodeHandle node("~");

  // receive planning problem
  ros::Subscriber map_sub = node.subscribe("/laser_cloud_surround", 1, mapCallback);
  ros::Subscriber sg_sub = node.subscribe("/start_goal", 1, sgCallback);

  // inform benchmark problem node of finishing
  ros::Publisher finish_pub = node.advertise<std_msgs::Empty>("/finish_test", 1, true);

  srand(ros::Time::now().toSec());
  ros::Duration(0.5).sleep();

  have_goal = false, have_map = false;
  const int use_map_num = 10;
  int exp_num = 0;

  /* ---------- initialize planner manager ---------- */
  SDFMap::Ptr sdf_map;
  sdf_map.reset(new SDFMap);
  sdf_map->initMap(node);

  EDTEnvironment::Ptr edt_env;
  edt_env.reset(new EDTEnvironment);
  edt_env->setMap(sdf_map);

  vector<BsplineOptimizer::Ptr> bspline_optimizers;
  for (int i = 0; i < 10; ++i) {
    BsplineOptimizer::Ptr optimizer;
    optimizer.reset(new BsplineOptimizer);
    optimizer->setParam(node);
    optimizer->setEnvironment(edt_env);
    bspline_optimizers.push_back(optimizer);
  }

  TopologyPRM::Ptr topo_prm;
  topo_prm.reset(new TopologyPRM);
  topo_prm->setEnvironment(edt_env);
  topo_prm->init(node);

  FastPlannerManager::Ptr planner_manager;
  planner_manager.reset(new FastPlannerManager);
  planner_manager->setParam(node);
  planner_manager->setOptimizer(bspline_optimizers);
  planner_manager->setTopoFinder(topo_prm);
  planner_manager->setEnvironment(edt_env);

  PlanningVisualization::Ptr visualization;
  visualization.reset(new PlanningVisualization(node));

  /* ---------- main loop ---------- */
  while (ros::ok()) {
    /* wait for map and sg ready */
    while (ros::ok()) {
      if (have_map && have_goal) break;
      // ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
    cout << "[kgb]: Map and SG ok!" << endl;

    /* ---------- manage map ---------- */
    if (exp_num % use_map_num == 0) {
      sdf_map->resetBuffer();
      for (int i = 0; i < latest_cloud.points.size(); ++i) {
        pcl::PointXYZ pt = latest_cloud.points[i];
        Eigen::Vector3d obs_pt;
        obs_pt(0) = pt.x;
        obs_pt(1) = pt.y;
        obs_pt(2) = pt.z;
        sdf_map->setOccupied(obs_pt);
      }
      sdf_map->updateESDF3d();
    }

    /* ---------- manage start and goal ---------- */
    Eigen::Vector3d start_pt, start_vel, start_acc, end_pt, end_vel;
    start_pt(0) = start_x, start_pt(1) = start_y, start_pt(2) = start_z;
    end_pt(0) = goal_x, end_pt(1) = goal_y, end_pt(2) = goal_z;

    start_vel.setZero();
    start_acc.setZero();
    end_vel.setZero();

    /* ---------- generate reference traj and local replanning ---------- */
    double plan_time = 0.0;
    ros::Time t1 = ros::Time::now();

    bool success = planner_manager->benckmarkReplan(start_pt, start_vel, start_acc, end_pt, end_vel);

    ros::Time t2 = ros::Time::now();
    cout << "[topo PG-GTO]: time: " << (t2 - t1).toSec() << endl;
    plan_time = (t2 - t1).toSec();

    if (!success) {
      cout << "[topo PG-GTO]: fail." << endl;
    } else {
      // retrieve and visualize
      planner_manager->updateTrajectoryInfo();

      visualization->drawPolynomialTraj(planner_manager->traj_manager_.global_traj_, 0.05,
                                        Eigen::Vector4d(0, 0, 0, 1), 0);
      visualization->drawBspline(planner_manager->position_traj_, 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1),
                                 false, 0.1, Eigen::Vector4d(1.0, 1.0, 1.0, 1), 99);
      visualization->drawTopoPathsPhase2(planner_manager->topo_select_paths_, 0.05);
      visualization->drawBsplinesPhase2(planner_manager->topo_traj_pos2_, 0.08);

      /* evaluation */
      double time_sum, length, mean_v, max_v, mean_a, max_a, jerk;
      double time_search, time_opt, time_adj;

      NonUniformBspline traj_opt = planner_manager->position_traj_;

      time_sum = traj_opt.getTimeSum();
      length = traj_opt.getLength();
      jerk = traj_opt.getJerk();
      traj_opt.getMeanAndMaxVel(mean_v, max_v);
      traj_opt.getMeanAndMaxAcc(mean_a, max_a);

      // traj_generator->getSolvingTime(time_search, time_opt, time_adj);
      // vector<double> vec_time, vec_cost;
      // traj_generator->getCostCurve(vec_cost, vec_time);
      // cout << "test3:" << exp_num + 1 << ",jerk:" << jerk;
      // cout << ",time:";
      // for (int i = 0; i < vec_time.size(); ++i)
      // {
      //   cout << vec_time[i];
      //   if (i != vec_time.size() - 1)
      //     cout << ";";
      // }
      // cout << ",cost:";
      // for (int i = 0; i < vec_cost.size(); ++i)
      // {
      //   cout << vec_cost[i];
      //   if (i != vec_cost.size() - 1)
      //     cout << ";";
      //   else
      //     cout << "\n";
      // }
      // std::ofstream
      // file("/home/bzhouai/workspaces/plan_ws/src/uav_planning_bm/resource/"
      //                    "back3.txt",
      //                    std::ios::app);
      // if (file.fail())
      // {
      //   cout << "open file error!\n";
      //   return -1;
      // }

      // file << "test3:" << exp_num + 1 << ",jerk:" << jerk;
      // file << ",time:";
      // for (int i = 0; i < vec_time.size(); ++i)
      // {
      //   file << vec_time[i];
      //   if (i != vec_time.size() - 1)
      //     file << ";";
      // }
      // file << ",cost:";
      // for (int i = 0; i < vec_cost.size(); ++i)
      // {
      //   file << vec_cost[i];
      //   if (i != vec_cost.size() - 1)
      //     file << ";";
      //   else
      //     file << "\n";
      // }

      // file.close();
    }

    /* finish test flag */
    cout << "[kgb]: finish test." << endl;
    ++exp_num;
    have_goal = false;
    if (exp_num % use_map_num == 0) have_map = false;

    std_msgs::Empty finish_msg;
    finish_pub.publish(finish_msg);
  }

  return 0;
}

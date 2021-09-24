#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
  class FastPlannerManager;
  class FastExplorationManager;
  class PlanningVisualization;
  struct FSMParam;
  struct FSMData;

  enum EXPL_STATE
  {
    INIT,
    PAUSED,
    PLAN_TRAJ,
    PUB_TRAJ,
    EXEC_TRAJ,
    FINISH
  };

  class FastExplorationFSM
  {
  private:
    /* planning utils */
    shared_ptr<FastPlannerManager> planner_manager_;
    shared_ptr<FastExplorationManager> expl_manager_;
    shared_ptr<PlanningVisualization> visualization_;

    shared_ptr<FSMParam> fp_;
    shared_ptr<FSMData> fd_;
    EXPL_STATE state_;

    bool classic_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
    ros::Subscriber start_sub_, pause_sub_, odom_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_;

    /* helper functions */
    int callExplorationPlanner();
    void transitState(EXPL_STATE new_state, string pos_call);

    /* ROS functions */
    void FSMCallback(const ros::TimerEvent &e);
    void safetyCallback(const ros::TimerEvent &e);
    void frontierCallback(const ros::TimerEvent &e);
    void startCallback(const std_msgs::Empty &msg);
    void pauseCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void visualize();
    void clearVisMarker();

  public:
    FastExplorationFSM(/* args */)
    {
    }
    ~FastExplorationFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace fast_planner

#endif
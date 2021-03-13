#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <bspline/non_uniform_bspline.h>
#include <poly_traj/polynomial_traj.h>
#include <path_searching/topo_prm.h>
#include <active_perception/traj_visibility.h>

using std::vector;

namespace fast_planner {
class GlobalTrajData {
private:
public:
  PolynomialTraj global_traj_;
  vector<NonUniformBspline> local_traj_;

  double global_duration_;
  ros::Time global_start_time_;
  double local_start_time_, local_end_time_;
  double time_change_;
  double last_time_inc_;

  GlobalTrajData(/* args */) {
  }

  ~GlobalTrajData() {
  }

  bool localTrajReachTarget() {
    return fabs(local_end_time_ - global_duration_) < 1e-3;
  }

  void setGlobalTraj(const PolynomialTraj& traj, const ros::Time& time) {
    global_traj_ = traj;
    global_duration_ = global_traj_.getTotalTime();
    global_start_time_ = time;

    local_traj_.clear();
    local_start_time_ = -1;
    local_end_time_ = -1;
    time_change_ = 0.0;
    last_time_inc_ = 0.0;
  }

  void setLocalTraj(const NonUniformBspline& traj, const double& local_ts, const double& local_te,
                    const double& time_change) {
    local_traj_.resize(3);
    local_traj_[0] = traj;
    local_traj_[1] = local_traj_[0].getDerivative();
    local_traj_[2] = local_traj_[1].getDerivative();

    local_start_time_ = local_ts;
    local_end_time_ = local_te;
    global_duration_ += time_change;
    time_change_ += time_change;
    last_time_inc_ = time_change;
  }

  Eigen::Vector3d getState(const double& t, const int& k) {
    if (t >= -1e-3 && t <= local_start_time_)
      return global_traj_.evaluate(t - time_change_ + last_time_inc_, k);
    else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      return global_traj_.evaluate(t - time_change_, k);
    else
      return local_traj_[k].evaluateDeBoorT(t - local_start_time_);
  }

  // Get data required to parameterize a Bspline within a duration
  void getTrajInfoInDuration(const double& start_t, const double& duration, const double& dt,
                             vector<Eigen::Vector3d>& point_set,
                             vector<Eigen::Vector3d>& start_end_derivative) {
    for (double tp = 0.0; tp <= duration + 1e-4; tp += dt) {
      auto cur_pt = getState(start_t + tp, 0);
      point_set.push_back(cur_pt);
    }
    start_end_derivative.push_back(getState(start_t, 1));
    start_end_derivative.push_back(getState(start_t + duration, 1));
    start_end_derivative.push_back(getState(start_t, 2));
    start_end_derivative.push_back(getState(start_t + duration, 2));
  }

  // Get data required to parameterize a Bspline within a sphere
  void getTrajInfoInSphere(const double& start_t, const double& radius, const double& dist_pt,
                           vector<Eigen::Vector3d>& point_set,
                           vector<Eigen::Vector3d>& start_end_derivative, double& dt, double& duration) {
    double segment_len = 0.0;                         // Length of the truncated segment
    double segment_time = 0.0;                        // Duration of the truncated segment
    Eigen::Vector3d first_pt = getState(start_t, 0);  // First point of the segment
    Eigen::Vector3d prev_pt = first_pt;
    Eigen::Vector3d cur_pt = first_pt;

    // Search the time at which distance to current position is larger than a threshold
    const double delta_t = 0.2;
    while ((cur_pt - first_pt).norm() < radius && start_t + segment_time < global_duration_ - 1e-3) {
      segment_time = min(segment_time + delta_t, global_duration_ - start_t);
      cur_pt = getState(start_t + segment_time, 0);
      segment_len += (cur_pt - prev_pt).norm();
      prev_pt = cur_pt;
    }

    // Get sampled state of the segment
    int seg_num = floor(segment_len / dist_pt);
    seg_num = max(6, seg_num);
    duration = segment_time;
    dt = duration / seg_num;
    getTrajInfoInDuration(start_t, duration, dt, point_set, start_end_derivative);
  }
};

struct PlanParameters {
  /* planning algorithm parameters */
  double max_vel_, max_acc_, max_jerk_;  // physical limits
  double accept_vel_, accept_acc_;

  double max_yawdot_;
  double local_traj_len_;  // local replanning trajectory length
  double ctrl_pt_dist;     // distance between adjacient B-spline control points
  int bspline_degree_;
  bool min_time_;

  double clearance_;
  int dynamic_;
  /* processing time */
  double time_search_ = 0.0;
  double time_optimize_ = 0.0;
  double time_adjust_ = 0.0;
};

struct LocalTrajData {
  /* info of generated traj */

  int traj_id_;
  double duration_;
  ros::Time start_time_;
  Eigen::Vector3d start_pos_;
  NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_, yaw_traj_, yawdot_traj_,
      yawdotdot_traj_;
};

// structure of trajectory info
struct LocalTrajState {
  Eigen::Vector3d pos, vel, acc;
  double yaw, yawdot;
  int id;
};

class LocalTrajServer {
private:
  LocalTrajData traj1_, traj2_;

public:
  LocalTrajServer(/* args */) {
    traj1_.traj_id_ = 0;
    traj2_.traj_id_ = 0;
  }
  ~LocalTrajServer() {
  }

  void addTraj(const LocalTrajData& traj) {
    if (traj1_.traj_id_ == 0) {
      // receive the first traj, save in traj1
      traj1_ = traj;
    } else {
      traj2_ = traj;
    }
  }

  bool evaluate(const ros::Time& time, LocalTrajState& traj_state) {
    if (traj1_.traj_id_ == 0) {
      // not receive traj yet
      return false;
    }

    if (traj2_.traj_id_ != 0 && time > traj2_.start_time_) {
      // have traj2 AND time within range of traj2. should use traj2 now
      traj1_ = traj2_;
      traj2_.traj_id_ = 0;
    }

    double t_cur = (time - traj1_.start_time_).toSec();
    if (t_cur < 0) {
      cout << "[Traj server]: invalid time." << endl;
      return false;
    } else if (t_cur < traj1_.duration_) {
      // time within range of traj 1
      traj_state.pos = traj1_.position_traj_.evaluateDeBoorT(t_cur);
      traj_state.vel = traj1_.velocity_traj_.evaluateDeBoorT(t_cur);
      traj_state.acc = traj1_.acceleration_traj_.evaluateDeBoorT(t_cur);
      traj_state.yaw = traj1_.yaw_traj_.evaluateDeBoorT(t_cur)[0];
      traj_state.yawdot = traj1_.yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      traj_state.id = traj1_.traj_id_;
      return true;
    } else {
      traj_state.pos = traj1_.position_traj_.evaluateDeBoorT(traj1_.duration_);
      traj_state.vel.setZero();
      traj_state.acc.setZero();
      traj_state.yaw = traj1_.yaw_traj_.evaluateDeBoorT(traj1_.duration_)[0];
      traj_state.yawdot = 0;
      traj_state.id = traj1_.traj_id_;
      return true;
    }
  }

  void resetDuration() {
    ros::Time now = ros::Time::now();
    if (traj1_.traj_id_ != 0) {
      double t_stop = (now - traj1_.start_time_).toSec();
      traj1_.duration_ = min(t_stop, traj1_.duration_);
    }
    if (traj2_.traj_id_ != 0) {
      double t_stop = (now - traj2_.start_time_).toSec();
      traj2_.duration_ = min(t_stop, traj2_.duration_);
    }
  }
};

class MidPlanData {
public:
  MidPlanData(/* args */) {
  }
  ~MidPlanData() {
  }

  vector<Eigen::Vector3d> global_waypoints_;

  // initial trajectory segment
  NonUniformBspline initial_local_segment_;
  vector<Eigen::Vector3d> local_start_end_derivative_;

  // kinodynamic path
  vector<Eigen::Vector3d> kino_path_;

  // topological paths
  list<GraphNode::Ptr> topo_graph_;
  vector<vector<Eigen::Vector3d>> topo_paths_;
  vector<vector<Eigen::Vector3d>> topo_filtered_paths_;
  vector<vector<Eigen::Vector3d>> topo_select_paths_;

  // multiple topological trajectories
  vector<NonUniformBspline> topo_traj_pos1_;
  vector<NonUniformBspline> topo_traj_pos2_;
  vector<NonUniformBspline> refines_;

  // visibility constraint
  vector<Eigen::Vector3d> block_pts_;
  Eigen::MatrixXd ctrl_pts_;
  NonUniformBspline no_visib_traj_;
  vector<VisiblePair> visib_pairs_;
  ViewConstraint view_cons_;

  // heading planning
  vector<vector<Eigen::Vector3d>> frontiers_;
  vector<double> path_yaw_;
  double dt_yaw_;
  double dt_yaw_path_;

  void clearTopoPaths() {
    topo_traj_pos1_.clear();
    topo_traj_pos2_.clear();
    topo_graph_.clear();
    topo_paths_.clear();
    topo_filtered_paths_.clear();
    topo_select_paths_.clear();
  }

  void addTopoPaths(list<GraphNode::Ptr>& graph, vector<vector<Eigen::Vector3d>>& paths,
                    vector<vector<Eigen::Vector3d>>& filtered_paths,
                    vector<vector<Eigen::Vector3d>>& selected_paths) {
    topo_graph_ = graph;
    topo_paths_ = paths;
    topo_filtered_paths_ = filtered_paths;
    topo_select_paths_ = selected_paths;
  }
};

}  // namespace fast_planner

#endif
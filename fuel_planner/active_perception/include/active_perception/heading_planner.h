#ifndef _HEADING_PLANNER_H_
#define _HEADING_PLANNER_H_

#include <vector>
#include <unordered_map>
#include <queue>
#include <list>
#include <memory>
#include <mutex>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

using std::list;
using std::queue;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

class RayCaster;

namespace fast_planner {
class SDFMap;

// Basic vertex type containing only general artributes required by graph search
class BaseVertex {
public:
  typedef shared_ptr<BaseVertex> Ptr;
  BaseVertex() {
  }
  ~BaseVertex() {
  }

  virtual void print() {
    std::cout << "no data in base vertex" << std::endl;
  }

  int id_;
  double g_value_;
};

// vertex type for heading planning

class YawVertex : public BaseVertex {
public:
  typedef shared_ptr<YawVertex> Ptr;
  YawVertex(const double& y, double gain, const int& id) {
    yaw_ = y;
    info_gain_ = gain;
    id_ = id;
  }
  ~YawVertex() {
  }
  virtual void print() {
    std::cout << "yaw: " << yaw_ << std::endl;
  }

  // vertex type specific members-------

  void printNeighbors() {
    for (auto v : neighbors_)
      v->print();
  }

  double gain(const YawVertex::Ptr& v) {
    return v->info_gain_;
  }

  double dist(const YawVertex::Ptr& v) {
    return fabs(yaw_ - v->yaw_);
  }

  list<YawVertex::Ptr> neighbors_;
  YawVertex::Ptr parent_;

  double yaw_;
  double info_gain_;

private:
  double visib_;

  // vertex type specific members-------
};

// Graph that can use different types of vertex
// with standard graph searching algorithm

// template <typename VERTEX>
class Graph {
public:
  Graph() {
  }
  ~Graph() {
  }

  void print();
  void addVertex(const YawVertex::Ptr& vertex);
  void addEdge(const int& from, const int& to);

  void setParams(const double& w, const double& my, const double& dt);
  void dijkstraSearch(const int& start, const int& goal, vector<YawVertex::Ptr>& path);

private:
  double penal(const double& diff);
  vector<YawVertex::Ptr> vertice_;
  double w_;
  double max_yaw_rate_;
  double dt_;
};

// !SECTION

class CastFlags {
private:
  /* data */
  vector<char> flags_;
  Eigen::Vector3i lb_, ub_, cells_;

public:
  CastFlags() {
  }
  CastFlags(const int& size) {
    flags_.resize(size);
  }
  ~CastFlags() {
  }

  void reset(const Eigen::Vector3i& lb, const Eigen::Vector3i& ub) {
    lb_ = lb;
    ub_ = ub;
    cells_ = ub_ - lb_;
    fill(flags_.begin(), flags_.end(), 0);
  }

  inline int address(const Eigen::Vector3i& idx) {
    Eigen::Vector3i diff = idx - lb_;
    return diff[2] + diff[1] * cells_[2] + diff[0] * cells_[1] * cells_[2];
  }

  inline char getFlag(const Eigen::Vector3i& idx) {
    return flags_[address(idx)];
  }

  inline void setFlag(const Eigen::Vector3i& idx, const char& f) {
    flags_[address(idx)] = f;
  }
};

class HeadingPlanner {
public:
  HeadingPlanner(ros::NodeHandle& nh);
  ~HeadingPlanner();

  void setMap(const shared_ptr<SDFMap>& map);
  void searchPathOfYaw(const vector<Eigen::Vector3d>& pts, const vector<double>& yaws, const double& dt,
                       const Eigen::MatrixXd& ctrl_pts, vector<double>& path);

  // frontier-based IG, not good to evaluate information gain
  void setFrontier(const vector<vector<Eigen::Vector3d>>& frontier);
  void calcVisibFrontier(const Eigen::Vector3d& pt, const double& yaw,
                         unordered_map<int, int>& visib_idx);
  void showVisibFrontier(const vector<YawVertex::Ptr>& path);
  double calcInfoGain(const Eigen::Vector3d& pt, const double& yaw, const int& task_id);

private:
  void setTransform(const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& t_wb);
  bool insideFoV(const Eigen::Vector4d& pw);

  // iterate within volume and check visibility, voexl are weighted by distance
  double calcInformationGain(const Eigen::Vector3d& pt, const double& yaw,
                             const Eigen::MatrixXd& ctrl_pts, const int& task_id);
  // iterate within volume and check visibility, voexl are weighted uniformly
  bool insideFoV(const Eigen::Vector3d& pw, const Eigen::Vector3d& pc,
                 const vector<Eigen::Vector3d>& normals);
  void distToPathAndCurPos(const Eigen::Vector3d& check_pt, const Eigen::MatrixXd& ctrl_pts,
                           std::pair<double, double>& dists, bool debug = false);
  void axisAlignedBoundingBox(const vector<Eigen::Vector3d>& points, Eigen::Vector3d& lb,
                              Eigen::Vector3d& ub);
  void visualizeBox(const Eigen::Vector3d& lb, const Eigen::Vector3d& ub);
  void calcFovAABB(const Eigen::Matrix3d& R_wc, const Eigen::Vector3d& t_wc, Eigen::Vector3i& lb,
                   Eigen::Vector3i& ub);
  void initCastFlag(const Eigen::Vector3d& pos);

  pcl::PointCloud<pcl::PointXYZ>::Ptr frontier_;
  pcl::KdTreeFLANN<pcl::PointXYZ> ft_kdtree_;

  shared_ptr<SDFMap> sdf_map_;
  vector<unique_ptr<RayCaster>> casters_;

  // normals of camera FoV seperating hyperplane
  Eigen::Vector3d n_top_, n_bottom_, n_left_, n_right_;
  // vertices of FoV
  Eigen::Vector3d lefttop_, righttop_, leftbottom_, rightbottom_;
  // flags for accelerated raycasting, 0: unvisited, 1: visible, 2: invisible
  CastFlags cast_flags_;
  // camera parameters
  double tanyz_, tanxz_, near_, far_;
  Eigen::Matrix4d T_cb_, T_bc_;  // transform between camera and body frame
  // debug
  ros::Publisher frontier_pub_, visib_pub_, box_pub_;
  // params
  double yaw_diff_, lambda1_, lambda2_;
  int half_vert_num_;
  double max_yaw_rate_, w_;

  enum WEIGHT_TYPE { NON_UNIFORM, UNIFORM };
  int weight_type_;
};

}  // namespace fast_planner

#endif
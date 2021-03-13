#include <active_perception/heading_planner.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <future>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

namespace fast_planner {
// template <typename VERTEX>
void Graph::print() {
  for (auto v : vertice_) {
    v->print();
    v->printNeighbors();
  }
}

// template <typename VERTEX>
void Graph::addVertex(const YawVertex::Ptr& vertex) {
  vertice_.push_back(vertex);
}

// template <typename VERTEX>
void Graph::addEdge(const int& from, const int& to) {
  vertice_[from]->neighbors_.push_back(vertice_[to]);
}

// template <typename VERTEX>
void Graph::setParams(const double& w, const double& my, const double& dt) {
  w_ = w;
  max_yaw_rate_ = my;
  dt_ = dt;
}

double Graph::penal(const double& diff) {
  double yr = diff / dt_;
  if (yr <= max_yaw_rate_) {
    return 0.0;
  } else {
    return pow(yr - max_yaw_rate_, 2);
  }
}

// template <typename VERTEX>
void Graph::dijkstraSearch(const int& start, const int& goal, vector<YawVertex::Ptr>& path) {
  YawVertex::Ptr start_v = vertice_[start];
  YawVertex::Ptr end_v = vertice_[goal];
  start_v->g_value_ = 0.0;

  queue<YawVertex::Ptr> open_set;
  unordered_map<int, int> open_set_map;
  unordered_map<int, int> close_set;
  open_set.push(start_v);
  open_set_map[start_v->id_] = 1;

  while (!open_set.empty()) {
    auto vc = open_set.front();
    open_set.pop();
    open_set_map.erase(vc->id_);
    close_set[vc->id_] = 1;

    // reach target
    if (vc == end_v) {
      // std::cout << "Dijkstra reach target" << std::endl;
      YawVertex::Ptr vit = vc;
      while (vit != nullptr) {
        path.push_back(vit);
        vit = vit->parent_;
      }
      reverse(path.begin(), path.end());
      return;
    }
    auto nbs = vc->neighbors_;
    for (auto vb : nbs) {
      // skip vertex in close set
      if (close_set.find(vb->id_) != close_set.end()) continue;

      // update new or open vertex
      double g_tmp = vc->g_value_ - vc->gain(vb) + w_ * penal(vc->dist(vb));
      if (open_set_map.find(vb->id_) == open_set_map.end()) {
        open_set_map[vb->id_] = 1;
        open_set.push(vb);
      } else if (g_tmp > vb->g_value_) {
        continue;
      }
      vb->parent_ = vc;
      vb->g_value_ = g_tmp;
    }
  }

  ROS_ERROR("Dijkstra can't find path!");
  ROS_ASSERT(false);
}

HeadingPlanner::HeadingPlanner(ros::NodeHandle& nh) {
  frontier_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/heading_planner/frontier", 20);
  visib_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/heading_planner/visib", 20);
  box_pub_ = nh.advertise<visualization_msgs::Marker>("/heading_planner/box", 20);

  nh.param("heading_planner/yaw_diff", yaw_diff_, -1.0);
  nh.param("heading_planner/lambda1", lambda1_, -1.0);
  nh.param("heading_planner/lambda2", lambda2_, -1.0);
  nh.param("heading_planner/half_vert_num", half_vert_num_, -1);
  nh.param("heading_planner/max_yaw_rate", max_yaw_rate_, -1.0);
  nh.param("heading_planner/w", w_, -1.0);
  nh.param("heading_planner/weight_type", weight_type_, -1);
  std::cout << "yaw diff: " << yaw_diff_ << std::endl;
  std::cout << "max yaw diff: " << max_yaw_rate_ << std::endl;
  std::cout << "vert num: " << half_vert_num_ << std::endl;

  // camera FoV params
  far_ = 4.5;
  // normals of hyperplanes
  const double top_ang = 0.56125;
  n_top_ << 0.0, sin(M_PI_2 - top_ang), cos(M_PI_2 - top_ang);
  n_bottom_ << 0.0, -sin(M_PI_2 - top_ang), cos(M_PI_2 - top_ang);

  const double left_ang = 0.69222;
  const double right_ang = 0.68901;
  n_left_ << sin(M_PI_2 - left_ang), 0.0, cos(M_PI_2 - left_ang);
  n_right_ << -sin(M_PI_2 - right_ang), 0.0, cos(M_PI_2 - right_ang);

  std::cout << "top: " << n_top_ << std::endl;
  std::cout << "bottom: " << n_bottom_ << std::endl;
  std::cout << "left: " << n_left_ << std::endl;
  std::cout << "right: " << n_right_ << std::endl;

  // vertices of FoV assuming zero pitch
  lefttop_ << -far_ * tan(left_ang), -far_ * sin(top_ang), far_;
  leftbottom_ << -far_ * sin(left_ang), far_ * sin(top_ang), far_;
  righttop_ << far_ * sin(right_ang), -far_ * sin(top_ang), far_;
  rightbottom_ << far_ * sin(right_ang), far_ * sin(top_ang), far_;

  std::cout << "lefttop: " << lefttop_.transpose() << std::endl;
  std::cout << "leftbottom: " << leftbottom_.transpose() << std::endl;
  std::cout << "righttop: " << righttop_.transpose() << std::endl;
  std::cout << "rightbottom: " << rightbottom_.transpose() << std::endl;

  cast_flags_ = CastFlags(1000000);
  // T_cb_ << 0, -1,  0, 0,
  //          0,  0,  1, 0,
  //          1,  0,  0, 0,
  //          0,  0,  0, 1;
  T_cb_ << 0, -1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  T_bc_ = T_cb_.inverse();

  std::cout << "T_cb: " << T_cb_ << std::endl;
  std::cout << "T_bc: " << T_bc_ << std::endl;

  casters_.resize(2 * half_vert_num_ + 1);
  for (int i = 0; i < 2 * half_vert_num_ + 1; ++i) {
    casters_[i].reset(new RayCaster);
  }
}

HeadingPlanner::~HeadingPlanner() {
}

void HeadingPlanner::setMap(const shared_ptr<SDFMap>& map) {
  sdf_map_ = map;
}

void HeadingPlanner::searchPathOfYaw(const vector<Eigen::Vector3d>& pts, const vector<double>& yaws,
                                     const double& dt, const Eigen::MatrixXd& ctrl_pts,
                                     vector<double>& path) {
  Graph yaw_graph;
  yaw_graph.setParams(w_, max_yaw_rate_, dt);
  int gid = 0;
  vector<YawVertex::Ptr> layer, last_layer;
  Eigen::Vector3d cur_pos = pts[0];

  for (int i = 0; i < yaws.size(); ++i) {
    // add one layer of vertice representing discretized yaws at one waypoint
    auto t1 = ros::Time::now();
    bool start_end = (i == 0 || i == yaws.size() - 1);
    if (start_end) {  // start and end vertice
      YawVertex::Ptr vert(new YawVertex(yaws[i], 0, gid++));
      yaw_graph.addVertex(vert);
      layer.push_back(vert);
    } else {  // inter vertice
      initCastFlag(pts[i]);
      int vert_num = 2 * half_vert_num_ + 1;
      vector<future<double>> futs(vert_num);
      for (int j = 0; j < vert_num; ++j) {  // evaluate info gain in parallel
        double ys = yaws[i] + double(j - half_vert_num_) * yaw_diff_;
        futs[j] = std::async(std::launch::async, &HeadingPlanner::calcInformationGain, this, pts[i], ys,
                             ctrl_pts, j);
      }
      for (int j = 0; j < vert_num; ++j) {
        double ys = yaws[i] + double(j - half_vert_num_) * yaw_diff_;
        double gain = futs[j].get();
        YawVertex::Ptr vert(new YawVertex(ys, gain, gid++));
        yaw_graph.addVertex(vert);
        layer.push_back(vert);
      }
    }
    // // debug
    // std::cout << "pos: " << pts[i].transpose() << std::endl;
    // std::cout << "center yaw: " << int(yaws[i] * 57.3) << std::endl;
    // for (auto vl : layer) {
    //   std::cout << vl->id_ << ":" << int(vl->yaw_ * 57.3) << "; ";
    // }
    // std::cout << "" << std::endl;
    // connect vertice from last layer to this layer
    // std::cout << "-------------" << (ros::Time::now() - t1).toSec() << " secs" <<
    // std::endl;

    for (auto v1 : last_layer) {
      for (auto v2 : layer) {
        yaw_graph.addEdge(v1->id_, v2->id_);
        // std::cout << v1->id_ << "->" << v2->id_ << "; ";
      }
    }
    last_layer.clear();
    last_layer.swap(layer);
  }
  auto t1 = ros::Time::now();
  vector<YawVertex::Ptr> vert_path;
  yaw_graph.dijkstraSearch(0, gid - 1, vert_path);

  std::cout << "Gains of " << vert_path.size() << " vertice: ";
  for (auto vert : vert_path) {
    path.push_back(vert->yaw_);
    std::cout << vert->info_gain_ << ", ";
  }
  std::cout << "" << std::endl;
}

double HeadingPlanner::calcInformationGain(const Eigen::Vector3d& pt, const double& yaw,
                                           const Eigen::MatrixXd& ctrl_pts, const int& task_id) {
  // compute camera transform
  auto t1 = ros::Time::now();
  Eigen::Matrix3d R_wb;
  R_wb << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix4d T_wb = Eigen::Matrix4d::Identity();
  T_wb.block(0, 0, 3, 3) = R_wb;
  T_wb.block(0, 3, 3, 1) = pt;
  Eigen::Matrix4d T_wc = T_wb * T_bc_;
  Eigen::Matrix3d R_wc = T_wc.block(0, 0, 3, 3);
  Eigen::Vector3d t_wc = T_wc.block(0, 3, 3, 1);

  // rotate camera seperating plane normals
  vector<Eigen::Vector3d> normals = { n_top_, n_bottom_, n_left_, n_right_ };
  for (auto& n : normals)
    n = R_wc * n;
  Eigen::Vector3i lbi, ubi;
  calcFovAABB(R_wc, t_wc, lbi, ubi);

  Eigen::Vector3i pt_idx, ray_id;
  Eigen::Vector3d check_pt, ray_pt;
  double resolution = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  Eigen::Vector3d offset = Eigen::Vector3d(0.5, 0.5, 0.5) - origin / resolution;
  pcl::PointCloud<pcl::PointXYZ> gain_pts;
  pair<double, double> dist12;
  const int factor = 4;
  double gain = 0;

  for (int x = lbi[0]; x <= ubi[0]; ++x)
    for (int y = lbi[1]; y <= ubi[1]; ++y)
      for (int z = lbi[2]; z <= ubi[2]; ++z) {
        // subsampling
        if (!(x % factor == 0 && y % factor == 0 && z % factor == 0)) continue;
        // check visibility of unknown cells in FOV, 1: accessible, 2: blocked
        pt_idx << x, y, z;
        if (!sdf_map_->getOccupancy(pt_idx) == SDFMap::UNKNOWN) continue;
        if (!sdf_map_->isInBox(pt_idx)) continue;
        sdf_map_->indexToPos(pt_idx, check_pt);
        if (!insideFoV(check_pt, pt, normals)) continue;

        char flag = cast_flags_.getFlag(pt_idx);
        if (flag == 1) {  // visited cell, fetch visibility directly
          if (weight_type_ == UNIFORM) {
            gain += 1;
          } else if (weight_type_ == NON_UNIFORM) {
            distToPathAndCurPos(check_pt, ctrl_pts, dist12);
            gain += exp(-lambda1_ * dist12.first - lambda2_ * dist12.second);
          }
        } else if (flag == 0) {  // unvisited cell, should raycast
          char result = 1;
          casters_[task_id]->setInput(check_pt / resolution, pt / resolution);
          while (casters_[task_id]->step(ray_pt)) {
            ray_id(0) = ray_pt(0) + offset(0);
            ray_id(1) = ray_pt(1) + offset(1);
            ray_id(2) = ray_pt(2) + offset(2);
            if (sdf_map_->getOccupancy(ray_id) == SDFMap::OCCUPIED) {
              result = 2;
              break;
            }
          }
          if (result == 1) {
            if (weight_type_ == UNIFORM) {
              gain += 1;
            } else if (weight_type_ == NON_UNIFORM) {
              distToPathAndCurPos(check_pt, ctrl_pts, dist12);
              gain += exp(-lambda1_ * dist12.first - lambda2_ * dist12.second);
            }
          }
          cast_flags_.setFlag(pt_idx, result);
        }
      }
  // double dist = (cur_pos - pos).norm();
  // gain = gain * exp(-lambda_ * dist);
  // sensor_msgs::PointCloud2 msg;
  // pcl::toROSMsg(gain_pts, msg);
  // msg.header.frame_id = "world";
  // visib_pub_.publish(msg);

  // ROS_WARN("gain %d is %lf, cost %lf secs", task_id, gain, (ros::Time::now() -
  // t1).toSec());
  // std::cout << "gain: " << gain << std::endl;
  return gain;
}

double HeadingPlanner::calcInfoGain(const Eigen::Vector3d& pt, const double& yaw, const int& task_id) {
  // compute camera transform
  auto t1 = ros::Time::now();
  Eigen::Matrix3d R_wb;
  R_wb << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix4d T_wb = Eigen::Matrix4d::Identity();
  T_wb.block(0, 0, 3, 3) = R_wb;
  T_wb.block(0, 3, 3, 1) = pt;
  Eigen::Matrix4d T_wc = T_wb * T_bc_;
  Eigen::Matrix3d R_wc = T_wc.block(0, 0, 3, 3);
  Eigen::Vector3d t_wc = T_wc.block(0, 3, 3, 1);

  // rotate camera seperating plane normals
  vector<Eigen::Vector3d> normals = { n_top_, n_bottom_, n_left_, n_right_ };
  for (auto& n : normals)
    n = R_wc * n;
  Eigen::Vector3i lbi, ubi;
  calcFovAABB(R_wc, t_wc, lbi, ubi);

  Eigen::Vector3i pt_idx, ray_id;
  Eigen::Vector3d check_pt, ray_pt;
  double resolution = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  Eigen::Vector3d offset = Eigen::Vector3d(0.5, 0.5, 0.5) - origin / resolution;
  pcl::PointCloud<pcl::PointXYZ> gain_pts;
  pair<double, double> dist12;
  const int factor = 4;
  double gain = 0;

  for (int x = lbi[0]; x <= ubi[0]; ++x) {
    for (int y = lbi[1]; y <= ubi[1]; ++y) {
      for (int z = lbi[2]; z <= ubi[2]; ++z) {
        // subsampling
        if (!(x % factor == 0 && y % factor == 0 && z % factor == 0)) continue;
        // check visibility of unknown cells in FOV, 1: accessible, 2: blocked
        pt_idx << x, y, z;
        if (!sdf_map_->getOccupancy(pt_idx) == SDFMap::UNKNOWN) continue;
        sdf_map_->indexToPos(pt_idx, check_pt);
        if (!insideFoV(check_pt, pt, normals)) continue;

        bool visible = 1;
        casters_[task_id]->setInput(check_pt / resolution, pt / resolution);
        while (casters_[task_id]->step(ray_pt)) {
          ray_id(0) = ray_pt(0) + offset(0);
          ray_id(1) = ray_pt(1) + offset(1);
          ray_id(2) = ray_pt(2) + offset(2);
          if (sdf_map_->getOccupancy(ray_id) == SDFMap::OCCUPIED) {
            visible = false;
            break;
          }
        }
        if (visible) gain += 1;
      }
    }
  }
  // double dist = (cur_pos - pos).norm();
  // gain = gain * exp(-lambda_ * dist);
  // sensor_msgs::PointCloud2 msg;
  // pcl::toROSMsg(gain_pts, msg);
  // msg.header.frame_id = "world";
  // visib_pub_.publish(msg);

  // ROS_WARN("gain %d is %lf, cost %lf secs", task_id, gain, (ros::Time::now() -
  // t1).toSec());
  return gain;
}

void HeadingPlanner::initCastFlag(const Eigen::Vector3d& pos) {
  Eigen::Vector3d vec(far_, far_, rightbottom_[1]);
  Eigen::Vector3i lbi, ubi;
  sdf_map_->posToIndex(pos - vec, lbi);
  sdf_map_->posToIndex(pos + vec, ubi);
  cast_flags_.reset(lbi, ubi);
}

void HeadingPlanner::calcFovAABB(const Eigen::Matrix3d& R_wc, const Eigen::Vector3d& t_wc,
                                 Eigen::Vector3i& lb, Eigen::Vector3i& ub) {
  // axis-aligned bounding box(AABB) of camera FoV
  vector<Eigen::Vector3d> vertice(5);
  vertice[0] = R_wc * lefttop_ + t_wc;
  vertice[1] = R_wc * leftbottom_ + t_wc;
  vertice[2] = R_wc * righttop_ + t_wc;
  vertice[3] = R_wc * rightbottom_ + t_wc;
  vertice[4] = t_wc;

  Eigen::Vector3d lbd, ubd;
  axisAlignedBoundingBox(vertice, lbd, ubd);
  sdf_map_->boundBox(lbd, ubd);
  // visualizeBox(lbd, ubd);

  sdf_map_->posToIndex(lbd, lb);
  sdf_map_->posToIndex(ubd, ub);
}

void HeadingPlanner::visualizeBox(const Eigen::Vector3d& lb, const Eigen::Vector3d& ub) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  Eigen::Vector3d pos = 0.5 * (lb + ub);
  Eigen::Vector3d scale = ub - lb;

  mk.pose.position.x = pos(0);
  mk.pose.position.y = pos(1);
  mk.pose.position.z = pos(2);

  mk.scale.x = scale(0);
  mk.scale.y = scale(1);
  mk.scale.z = scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  box_pub_.publish(mk);
}

void HeadingPlanner::distToPathAndCurPos(const Eigen::Vector3d& check_pt,
                                         const Eigen::MatrixXd& ctrl_pts, pair<double, double>& dists,
                                         bool debug) {
  double min_squ = numeric_limits<double>::max();
  int idx = -1;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    Eigen::Vector3d ctrl_pt = ctrl_pts.row(i);
    double squ = (ctrl_pt - check_pt).squaredNorm();
    if (squ < min_squ) {
      min_squ = squ;
      idx = i;
    }
  }
  dists.first = sqrt(min_squ);
  dists.second = 0.0;
  for (int i = 0; i < idx; ++i) {
    dists.second += (ctrl_pts.row(i + 1) - ctrl_pts.row(i)).norm();
  }
  if (debug)
    std::cout << "pos: " << check_pt.transpose() << ", d1: " << dists.first << ", d2: " << dists.second
              << std::endl;
}

bool HeadingPlanner::insideFoV(const Eigen::Vector3d& pw, const Eigen::Vector3d& pc,
                               const vector<Eigen::Vector3d>& normals) {
  Eigen::Vector3d dir = pw - pc;
  if (dir.norm() > far_) {
    return false;
  }
  for (auto n : normals) {
    if (dir.dot(n) < 0.1) {
      return false;
    }
  }
  return true;
}

void HeadingPlanner::setFrontier(const vector<vector<Eigen::Vector3d>>& frontier) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_input(new pcl::PointCloud<pcl::PointXYZ>);
  frontier_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  for (const auto& ft : frontier) {
    for (const auto& p : ft)
      raw_input->push_back(pcl::PointXYZ(p[0], p[1], p[2]));
  }

  // downsample use voxel grid filter
  std::cout << "num1: " << raw_input->points.size() << std::endl;

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(raw_input);
  sor.setLeafSize(0.2f, 0.2f, 0.2f);
  sor.filter(*frontier_);

  std::cout << "num2: " << frontier_->points.size() << std::endl;

  frontier_->width = frontier_->points.size();
  frontier_->height = 1;
  frontier_->is_dense = true;
  frontier_->header.frame_id = "world";

  ft_kdtree_.setInputCloud(frontier_);
}

void HeadingPlanner::calcVisibFrontier(const Eigen::Vector3d& pt, const double& yaw,
                                       unordered_map<int, int>& visib_idx) {
  auto t1 = ros::Time::now();

  // search relevant frontier points
  pcl::PointXYZ search_pt(pt[0], pt[1], pt[2]);
  std::vector<int> point_idx;
  std::vector<float> point_radius;

  int num = ft_kdtree_.radiusSearch(search_pt, far_ + 1.0, point_idx, point_radius);
  if (num == 0) {
    return;
  }
  // std::cout << "radius search num: " << num << std::endl;

  // compute camera transform
  Eigen::Matrix3d R_wc;
  R_wc << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;

  setTransform(R_wc, pt);

  // find points lying inside the sensor FoV
  // pcl::PointCloud<pcl::PointXYZ> visib_pts;

  for (auto& idx : point_idx) {
    auto pt = frontier_->points[idx];

    Eigen::Vector4d pw(pt.x, pt.y, pt.z, 1);
    if (insideFoV(pw)) {
      // visib_pts.push_back(pt);
      visib_idx[idx] = 1;
    }
  }

  // visib_pts.width = visib_pts.points.size();
  // visib_pts.height = 1;
  // visib_pts.is_dense = true;
  // visib_pts.header.frame_id = "world";

  // // visualize to debug
  // sensor_msgs::PointCloud2 msg;
  // pcl::toROSMsg(visib_pts, msg);
  // visib_pub_.publish(msg);

  // std::cout << "visib fronter time: " << (ros::Time::now() - t1).toSec() <<
  // std::endl; std::cout << "visib size: " << visib_pts.points.size() << std::endl;
}

void HeadingPlanner::showVisibFrontier(const vector<YawVertex::Ptr>& path) {
  // for (auto v : path) {
  //   std::cout << v->yaw_ << std::endl;
  // }

  // auto visib = path.back()->path_visib_;

  // pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  // for (auto vt : visib) {
  //   indices->indices.push_back(vt.first);
  // }

  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // pcl::PointCloud<pcl::PointXYZ> output1, output2;

  // extract.setInputCloud(frontier_);
  // extract.setIndices(indices);
  // extract.filter(output1);

  // sensor_msgs::PointCloud2 msg;
  // pcl::toROSMsg(output1, msg);
  // visib_pub_.publish(msg);

  // extract.setNegative(true);
  // extract.filter(output2);
  // pcl::toROSMsg(output2, msg);
  // frontier_pub_.publish(msg);

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*frontier_, msg);
  frontier_pub_.publish(msg);
}

void HeadingPlanner::axisAlignedBoundingBox(const vector<Eigen::Vector3d>& points, Eigen::Vector3d& lb,
                                            Eigen::Vector3d& ub) {
  lb = points.front();
  ub = points.front();
  for (auto p : points) {
    lb = lb.array().min(p.array());
    ub = ub.array().max(p.array());
  }
}

void HeadingPlanner::setTransform(const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& t_wb) {
  // R_cw_ = R_wc.transpose();
  // t_cw_ = -R_cw_ * t_wc;
  Eigen::Matrix4d T_bw = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R_bw = R_wb.transpose();

  T_bw.block(0, 0, 3, 3) = R_bw;
  T_bw.block(0, 3, 3, 1) = -R_bw * t_wb;

  // T_cw_ = T_cb_ * T_bw;
}

bool HeadingPlanner::insideFoV(const Eigen::Vector4d& pw) {
  // transform to camera frame
  // Eigen::Vector4d pc = T_cw_ * pw;
  Eigen::Vector4d pc;

  if (pc(2) < near_ || pc(2) > far_) {
    return false;
  }

  double txz = pc(0) / pc(2);
  if (txz > tanxz_ || txz < -tanxz_) {
    return false;
  }

  double tyz = pc(1) / pc(2);
  if (tyz > tanyz_ || tyz < -tanyz_) {
    return false;
  }

  return true;
}

}  // namespace fast_planner
#include <active_perception/perception_utils.h>

#include <pcl/filters/voxel_grid.h>

namespace fast_planner {
PerceptionUtils::PerceptionUtils(ros::NodeHandle& nh) {
  nh.param("perception_utils/top_angle", top_angle_, -1.0);
  nh.param("perception_utils/left_angle", left_angle_, -1.0);
  nh.param("perception_utils/right_angle", right_angle_, -1.0);
  nh.param("perception_utils/max_dist", max_dist_, -1.0);
  nh.param("perception_utils/vis_dist", vis_dist_, -1.0);

  n_top_ << 0.0, sin(M_PI_2 - top_angle_), cos(M_PI_2 - top_angle_);
  n_bottom_ << 0.0, -sin(M_PI_2 - top_angle_), cos(M_PI_2 - top_angle_);

  n_left_ << sin(M_PI_2 - left_angle_), 0.0, cos(M_PI_2 - left_angle_);
  n_right_ << -sin(M_PI_2 - right_angle_), 0.0, cos(M_PI_2 - right_angle_);
  T_cb_ << 0, -1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  T_bc_ = T_cb_.inverse();

  // FOV vertices in body frame, for FOV visualization
  double hor = vis_dist_ * tan(left_angle_);
  double vert = vis_dist_ * tan(top_angle_);
  Vector3d origin(0, 0, 0);
  Vector3d left_up(vis_dist_, hor, vert);
  Vector3d left_down(vis_dist_, hor, -vert);
  Vector3d right_up(vis_dist_, -hor, vert);
  Vector3d right_down(vis_dist_, -hor, -vert);

  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(left_up);
  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(left_down);
  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(right_up);
  cam_vertices1_.push_back(origin);
  cam_vertices2_.push_back(right_down);

  cam_vertices1_.push_back(left_up);
  cam_vertices2_.push_back(right_up);
  cam_vertices1_.push_back(right_up);
  cam_vertices2_.push_back(right_down);
  cam_vertices1_.push_back(right_down);
  cam_vertices2_.push_back(left_down);
  cam_vertices1_.push_back(left_down);
  cam_vertices2_.push_back(left_up);
}

void PerceptionUtils::setPose(const Vector3d& pos, const double& yaw) {
  pos_ = pos;
  yaw_ = yaw;

  // Transform the normals of camera FOV
  Eigen::Matrix3d R_wb;
  R_wb << cos(yaw_), -sin(yaw_), 0.0, sin(yaw_), cos(yaw_), 0.0, 0.0, 0.0, 1.0;
  Vector3d pc = pos_;

  Eigen::Matrix4d T_wb = Eigen::Matrix4d::Identity();
  T_wb.block<3, 3>(0, 0) = R_wb;
  T_wb.block<3, 1>(0, 3) = pc;
  Eigen::Matrix4d T_wc = T_wb * T_bc_;
  Eigen::Matrix3d R_wc = T_wc.block<3, 3>(0, 0);
  // Vector3d t_wc = T_wc.block<3, 1>(0, 3);
  normals_ = { n_top_, n_bottom_, n_left_, n_right_ };
  for (auto& n : normals_)
    n = R_wc * n;
}

void PerceptionUtils::getFOV(vector<Vector3d>& list1, vector<Vector3d>& list2) {
  list1.clear();
  list2.clear();

  // Get info for visualizing FOV at (pos, yaw)
  Eigen::Matrix3d Rwb;
  Rwb << cos(yaw_), -sin(yaw_), 0, sin(yaw_), cos(yaw_), 0, 0, 0, 1;
  for (int i = 0; i < cam_vertices1_.size(); ++i) {
    auto p1 = Rwb * cam_vertices1_[i] + pos_;
    auto p2 = Rwb * cam_vertices2_[i] + pos_;
    list1.push_back(p1);
    list2.push_back(p2);
  }
}

bool PerceptionUtils::insideFOV(const Vector3d& point) {
  Eigen::Vector3d dir = point - pos_;
  if (dir.norm() > max_dist_) return false;

  dir.normalize();
  for (auto n : normals_) {
    if (dir.dot(n) < 0.0) return false;
  }
  return true;
}

void PerceptionUtils::getFOVBoundingBox(Vector3d& bmin, Vector3d& bmax) {
  double left = yaw_ + left_angle_;
  double right = yaw_ - right_angle_;
  Vector3d left_pt = pos_ + max_dist_ * Vector3d(cos(left), sin(left), 0);
  Vector3d right_pt = pos_ + max_dist_ * Vector3d(cos(right), sin(right), 0);
  vector<Vector3d> points = { left_pt, right_pt };
  if (left > 0 && right < 0)
    points.push_back(pos_ + max_dist_ * Vector3d(1, 0, 0));
  else if (left > M_PI_2 && right < M_PI_2)
    points.push_back(pos_ + max_dist_ * Vector3d(0, 1, 0));
  else if (left > -M_PI_2 && right < -M_PI_2)
    points.push_back(pos_ + max_dist_ * Vector3d(0, -1, 0));
  else if ((left > M_PI && right < M_PI) || (left > -M_PI && right < -M_PI))
    points.push_back(pos_ + max_dist_ * Vector3d(-1, 0, 0));

  bmax = bmin = pos_;
  for (auto p : points) {
    bmax = bmax.array().max(p.array());
    bmin = bmin.array().min(p.array());
  }
}

}  // namespace fast_planner
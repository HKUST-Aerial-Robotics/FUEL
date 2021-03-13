#include <active_perception/traj_visibility.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>

namespace fast_planner {
VisibilityUtil::VisibilityUtil() {
}

VisibilityUtil::VisibilityUtil(const ros::NodeHandle& nh) {
  // set parameters
  nh.param("visibility/visib_min", min_visib_, -1.0);
  nh.param("visibility/max_safe_dist", max_safe_dist_, -1.0);
  nh.param("visibility/safe_margin", safe_margin_, -1.0);
  nh.param("visibility/max_acc", max_acc_, -1.0);
  nh.param("visibility/r0", r0_, -1.0);
  nh.param("optimization/wnl", wnl_, -1.0);
  nh.param("visibility/forward", forward_, -1.0);
  caster_.reset(new RayCaster);
}

VisibilityUtil::~VisibilityUtil() {
}

void VisibilityUtil::setEDTEnvironment(const EDTEnvironment::Ptr& edt) {
  edt_env_ = edt;
  resolution_ = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);
  offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - origin / resolution_;
}

Eigen::Vector3d VisibilityUtil::getMinDistVoxel(const Eigen::Vector3d& q1, const Eigen::Vector3d& q2,
                                                const Eigen::Vector3d& offset, const double& res) {
  Eigen::Vector3d qk, min_pt;
  Eigen::Vector3i pt_id, min_id;
  double min_dist = 1000000, dist;

  caster_->setInput(q1 / res, q2 / res);

  while (caster_->step(qk)) {
    pt_id(0) = qk(0) + offset(0);
    pt_id(1) = qk(1) + offset(1);
    pt_id(2) = qk(2) + offset(2);

    dist = edt_env_->sdf_map_->getDistance(pt_id);

    if (dist < min_dist) {
      // projection should be on viewing ray
      min_dist = dist;
      min_id = pt_id;
    }
  }

  edt_env_->sdf_map_->indexToPos(min_id, min_pt);
  return min_pt;
}

Eigen::Vector3d VisibilityUtil::getMinDistVoxelOnLine(const Eigen::Vector3d& q1,
                                                      const Eigen::Vector3d& q2,
                                                      const Eigen::Vector3d& offset, const double& res,
                                                      int& state, Eigen::Vector3d& block) {
  Eigen::Vector3d tmp;
  Eigen::Vector3i pt_id;
  double min_dist = 1000000, dist;
  Eigen::Vector3d qb, grad, min_pt;
  Eigen::Vector3d dir = (q2 - q1).normalized();
  double norm = (q2 - q1).norm();
  state = 0;

  caster_->setInput(q1 / res, q2 / res);
  while (caster_->step(tmp)) {
    pt_id(0) = tmp(0) + offset(0);
    pt_id(1) = tmp(1) + offset(1);
    pt_id(2) = tmp(2) + offset(2);
    dist = edt_env_->sdf_map_->getDistance(pt_id);

    if (dist < min_dist) {
      edt_env_->sdf_map_->indexToPos(pt_id, tmp);
      if (dist < 1e-3) {
        min_dist = dist;
        min_pt = tmp;
        state = -1;
      } else if (dist < 1.2) {
        // projection on the line?
        edt_env_->evaluateEDTWithGrad(tmp, -1, dist, grad);
        if (grad.norm() > 1e-3) {
          qb = tmp - grad.normalized() * dist;
          double proj = (qb - q1).dot(dir) / norm;
          if (proj > 0.01 && proj < 0.99) {
            min_dist = dist;
            min_pt = tmp;
            state = 1;
            block = qb;
          }
        }
      }
    }
  }
  return min_pt;
}

bool VisibilityUtil::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                               Eigen::Vector3d& pc) {
  Eigen::Vector3d ray_pt, grad, pt;
  Eigen::Vector3i pt_id;
  double dist;

  caster_->setInput(p1 / resolution_, p2 / resolution_);
  while (caster_->step(ray_pt)) {
    pt_id(0) = ray_pt(0) + offset_(0);
    pt_id(1) = ray_pt(1) + offset_(1);
    pt_id(2) = ray_pt(2) + offset_(2);
    edt_env_->sdf_map_->indexToPos(pt_id, pt);
    edt_env_->evaluateEDTWithGrad(pt, -1, dist, grad);
    if (dist <= min_visib_) {
      edt_env_->sdf_map_->indexToPos(pt_id, pc);
      return false;
    }
  }
  return true;
}

void VisibilityUtil::findVisibPairs(const vector<Eigen::Vector3d>& pts, vector<VisiblePair>& pairs) {
  int know_num = 0;
  pairs.clear();
  int cur_j, prev_j = -1;
  for (int i = 0; i < pts.size() - visible_num_; ++i) {
    Eigen::Vector3d qi = pts[i];
    Eigen::Vector3d qj, qb;
    // find the first qj not seen by qi
    cur_j = -1;
    for (int j = i + visible_num_; j < pts.size(); ++j) {
      qj = pts[j];
      if (!lineVisib(qi, qj, qb)) {
        cur_j = j;
        break;
      }
    }
    if (cur_j == -1) break;  // all points visible, no need to pair

    VisiblePair vpair;
    vpair.from_ = i;
    vpair.to_ = cur_j;
    Eigen::Vector3d grad;
    double dist;
    edt_env_->evaluateEDTWithGrad(qb, -1, dist, grad);
    if (grad.norm() > 1e-3) {
      grad.normalize();
      Eigen::Vector3d dir = (qj - qi).normalized();
      Eigen::Vector3d push = grad - grad.dot(dir) * dir;
      push.normalize();
      vpair.qb_ = qb - 0.2 * push;
    }
    pairs.push_back(vpair);

    if (cur_j != prev_j && prev_j != -1) break;
    prev_j = cur_j;
  }
}

bool VisibilityUtil::findUnknownPoint(NonUniformBspline& traj, Eigen::Vector3d& point, double& time) {
  Eigen::Vector3d pt = traj.evaluateDeBoorT(0.0);
  if (edt_env_->sdf_map_->getOccupancy(pt) == SDFMap::UNKNOWN) {  // no initialization of map
    return false;
  }
  double duration = traj.getTimeSum();
  bool found = false;
  for (double t = 0.05; t <= duration + 1e-3; t += 0.05) {
    pt = traj.evaluateDeBoorT(t);
    if (edt_env_->sdf_map_->getOccupancy(pt) == SDFMap::UNKNOWN) {
      found = true;
      point = pt;
      time = t;
      break;
    }
  }
  if (!found) return false;  // all points are visible

  // go a little bit forward
  double forward = 0.0;
  Eigen::Vector3d prev = point;
  for (double t = time + 0.05; t <= duration + 1e-3; t += 0.05) {
    Eigen::Vector3d cur = traj.evaluateDeBoorT(t);
    forward += (cur - prev).norm();
    if (forward > forward_) {
      point = cur;
      time = t;
      break;
    }
    prev = cur;
  }
  // ROS_WARN_STREAM("unknown: " << point.transpose());
  return true;
}

bool VisibilityUtil::findCriticalPoint(NonUniformBspline& traj, const Eigen::Vector3d& unknown_pt,
                                       const double& unknown_t, Eigen::Vector3d& pc, double& tc) {
  Eigen::Vector3d pt, pb;
  double tb = -10.0;
  // coarse finding backward
  for (double t = unknown_t - 0.2; t >= 1e-3; t -= 0.2) {
    Eigen::Vector3d pt = traj.evaluateDeBoorT(t);
    if (!lineVisib(unknown_pt, pt, pb)) {
      tb = t;
      break;
    }
  }
  if (tb < -5) {
    std::cout << "all pt visible" << std::endl;
    return false;
  }

  // fine finding forward
  for (double t = tb + 0.01; t <= unknown_t + 1e-3; t += 0.01) {
    pt = traj.evaluateDeBoorT(t);
    if (lineVisib(unknown_pt, pt, pb)) {
      break;
    }
    tc = t;
    pc = pt;
  }
  return true;
}

bool VisibilityUtil::findDirAndIdx(NonUniformBspline& traj, const double& unknown_t,
                                   const double& crit_t, Eigen::Vector3d& dir, int& idx,
                                   Eigen::Vector3d& min_pt) {
  // max speed to unknown pt
  // double mean_v = 0.0;
  // int num_v = 0;
  double max_v;
  auto vel = traj.getDerivative();
  for (double t = 0; t <= unknown_t; t += 0.1) {
    double vt = vel.evaluateDeBoorT(t).norm();
    max_v = max(max_v, vt);
    // mean_v += vt;
    // ++num_v;
  }
  double dist1 = r0_ + forward_ + pow(max_v, 2) / (2 * max_acc_);
  std::cout << "max v: " << max_v << ", d: " << dist1 << std::endl;
  // mean_v = mean_v / num_v;
  // std::cout << "mean v: " << mean_v << ", d1: " << dist1 << std::endl;
  // double cur_v = vel.evaluateDeBoorT(0.0).norm();
  // double dist0 = r0_ + pow(max_v, 2) / (2 * max_acc_);

  // find the maximum safe distance along view direction
  Eigen::Vector3d unknown = traj.evaluateDeBoorT(unknown_t);
  Eigen::Vector3d critic = traj.evaluateDeBoorT(crit_t);
  Eigen::Vector3d v = (critic - unknown).normalized();
  for (double n = 0.0; n <= dist1 + 1e-3; n += 0.1) {
    Eigen::Vector3d pv = unknown + v * n;
    if (edt_env_->sdf_map_->getDistance(pv) <= resolution_) {
      dist1 = n - safe_margin_;
      break;
    }
  }
  dist1 = min(max_safe_dist_, dist1);
  dir = v * dist1;
  std::cout << "dir: " << dir.transpose() << ", len: " << dir.norm() << std::endl;

  // find min cost idx
  double dt = traj.getKnotSpan();
  double min_cost = 1000000;
  int idu = unknown_t / dt + 2;
  int idb = crit_t / dt + 1;
  Eigen::MatrixXd pts = traj.getControlPoint();

  for (int i = idu; i >= 3; --i) {
    Eigen::Vector3d pt = pts.row(i);
    Eigen::Vector3d dl = ((pt - unknown).dot(v)) * v;
    Eigen::Vector3d dn = (pt - unknown) - dl;
    double cost = dn.squaredNorm();
    if (dl.norm() < dist1) {
      cost += wnl_ * pow(dl.norm() - dist1, 2);
    }
    if (cost < min_cost) {
      min_cost = cost;
      idx = i;
      min_pt = pt;
    }
  }
  if (idx > idb) {
    std::cout << "min pt in front of critic" << std::endl;
    return false;
  }
  std::cout << "idx: " << idx << std::endl;
}

void VisibilityUtil::calcViewConstraint(NonUniformBspline& traj, ViewConstraint& cons) {
  // ROS_WARN("find visib constraint");
  cons.idx_ = -1;
  Eigen::Vector3d unknown, critic, dir, min_pt;
  double unknown_t, crit_t;
  int idx;

  if (!findUnknownPoint(traj, unknown, unknown_t)) return;

  if (!findCriticalPoint(traj, unknown, unknown_t, critic, crit_t)) return;

  if (!findDirAndIdx(traj, unknown_t, crit_t, dir, idx, min_pt)) return;

  cons.pt_ = unknown;
  cons.pc_ = critic;
  cons.dir_ = dir;
  cons.idx_ = idx;
  cons.pcons_ = min_pt;
  // ROS_WARN_STREAM("[view] unknown pt: " << cons.pt_.transpose() << ", dir: " << cons.dir_.transpose()
  //                                       << ", idx: " << cons.idx_);
}

/* precompute the blocking points of the trajectory which will be used for
 * evaluating the visibility cost */
vector<Eigen::Vector3d> VisibilityUtil::precomputeForVisibility(const vector<Eigen::Vector3d>& ctrl_pts,
                                                                bool debug) {
  int n = ctrl_pts.size() - visible_num_;
  vector<Eigen::Vector3d> block_pts(n);
  int unknwon_num = 0;
  double res = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);

  Eigen::Vector3d offset = Eigen::Vector3d(0.5, 0.5, 0.5) - origin / res;

  for (int j = ctrl_pts.size() - 1; j >= visible_num_; --j) {
    // if in known space for some times, skip
    // TODO: better condition of unknown space?
    Eigen::Vector3d qj = ctrl_pts[j];
    int i = j - visible_num_;
    double d = edt_env_->sdf_map_->getDistance(qj);
    if (d < 9999) {
      if (unknwon_num < 5) {
        ++unknwon_num;
      } else {
        block_pts[i](2) = -10086;
        continue;
      }
    }
    // method 1:
    Eigen::Vector3d qi = ctrl_pts[i], block;
    int state;
    Eigen::Vector3d min_pt = getMinDistVoxelOnLine(qi, qj, offset, res, state, block);
    if (state == 1) {
      block_pts[i] = block;
    } else if (state == -1) {
      block_pts[i] = getVirtualBlockPt(ctrl_pts, i, j, min_pt);
    } else if (state == 0) {
      block_pts[i](2) = -10086;
    } else {
      ROS_ERROR("WHAT?");
    }

    // // method 2:
    // Eigen::Vector3d qi = ctrl_pts[i];
    // Eigen::Vector3d min_pt = getMinDistVoxel(qi, qj, offset, res);
    // double dist;
    // Eigen::Vector3d grad;
    // edt_env_->evaluateEDTWithGrad(min_pt, -1, dist, grad);

    // // add view blocking point if min distance is negative, or its projection
    // // on the line segment qiqj
    // if (grad.norm() > 1e-3) {
    //   if (dist < 1e-3) {
    //     block_pts[i] = getVirtualBlockPt(ctrl_pts, i, j, min_pt);
    //   } else {
    //     Eigen::Vector3d qb = min_pt - grad.normalized() * dist;
    //     double proj = (qb - qi).dot((qj - qi).normalized()) / (qj -
    //     qi).norm(); if (proj > 0.01 && proj < 0.99 && dist < 3.0) {
    //       block_pts[i] = qb;
    //     } else {
    //       block_pts[i](2) = -10086;
    //       if (debug) {
    //         std::cout << "out: " << j << "/" << ctrl_pts.size() - 1
    //                   << std::endl;
    //       }
    //     }
    //   }
    // } else {
    //   block_pts[i](2) = -10086;
    //   if (debug) {
    //     std::cout << "grad: " << j << "/" << ctrl_pts.size() - 1 <<
    //     std::endl;
    //   }
    // }
  }

  return block_pts;
}

Eigen::Vector3d VisibilityUtil::getVirtualBlockPt(const vector<Eigen::Vector3d>& q, int i, int j,
                                                  const Eigen::Vector3d& min_pt) {
  Eigen::Vector3d qij = q[j] - q[i];
  Eigen::Vector3d qkm, qk;
  double min_dot = 1000;

  for (int k = i; k <= j; ++k) {
    qkm = q[k] - min_pt;
    double dot = fabs(qkm.dot(qij));
    if (dot < min_dot) {
      min_dot = dot;
      qk = q[k];
    }
  }

  // project and offset
  Eigen::Vector3d dir = qij.normalized();
  Eigen::Vector3d qp = q[i] + dir * (qk - q[i]).dot(dir);
  Eigen::Vector3d qv = qp + (qp - qk).normalized() * 0.1;
  return qv;
}

}  // namespace fast_planner
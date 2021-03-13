#ifndef _POLYNOMIAL_TRAJ_H
#define _POLYNOMIAL_TRAJ_H

#include <Eigen/Eigen>
#include <vector>

using std::vector;

namespace fast_planner {
// A segment of polynomial trajectory
class Polynomial {
public:
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  Polynomial() {
  }
  ~Polynomial() {
  }

  Polynomial(const Vector6d& cx, const Vector6d& cy, const Vector6d& cz, const double& time) {
    cx_ = cx;
    cy_ = cy;
    cz_ = cz;
    time_ = time;
  }

  // Get the k-th order derivative of t^n, n*(n-1)...(n-k+1)* t^(n-k)
  double getTBasis(const double& t, const int& n, const int& k) {
    int coeff = 1;
    for (int i = n; i >= n - k + 1; --i)
      coeff *= i;
    return coeff * pow(t, n - k);
  }

  // Evaluate pos, vel, acc..., k=0 for pos, 1 for vel...
  Eigen::Vector3d evaluate(const double& t, const int& k) {
    Vector6d tv = Vector6d::Zero();
    for (int i = k; i < 6; ++i)
      tv[i] = getTBasis(t, i, k);

    Eigen::Vector3d pt;
    pt[0] = tv.dot(cx_);
    pt[1] = tv.dot(cy_);
    pt[2] = tv.dot(cz_);
    return pt;
  }

  // Get the duration of the polynomial
  double getTime() const {
    return time_;
  }

private:
  // Time of the polynomial segment
  double time_;

  // Coefficient of x,y,z
  // 5 degree, p0 + p1*t + p2*t^2....
  Vector6d cx_, cy_, cz_;
};

class PolynomialTraj {
public:
  PolynomialTraj(/* args */) {
  }
  ~PolynomialTraj() {
  }

  void reset() {
    segments_.clear();
    times_.clear();
    sample_points_.clear();
    time_sum_ = -1;
    length_ = -1;
  }

  void addSegment(const Polynomial& poly) {
    segments_.push_back(poly);
    times_.push_back(poly.getTime());
  }

  Eigen::Vector3d evaluate(const double& t, const int& k) {
    // Find which segment t belong to
    int idx = 0;
    double ts = t;
    while (times_[idx] + 1e-4 < ts)
      ts -= times_[idx++];

    return segments_[idx].evaluate(ts, k);
  }

  double getTotalTime() {
    time_sum_ = 0.0;
    for (auto t : times_)
      time_sum_ += t;
    return time_sum_;
  }

  void getSamplePoints(vector<Eigen::Vector3d>& points) {
    double eval_t = 0.0;
    double total_t = getTotalTime();
    points.clear();
    while (eval_t < total_t) {
      Eigen::Vector3d pt = evaluate(eval_t, 0);
      points.push_back(pt);
      eval_t += 0.01;
    }
    sample_points_ = points;
  }

  double getLength() {
    vector<Eigen::Vector3d> pts;
    if (sample_points_.empty()) getSamplePoints(pts);

    length_ = 0.0;
    Eigen::Vector3d p_prev = sample_points_[0];
    Eigen::Vector3d p_cur;
    for (int i = 1; i < sample_points_.size(); ++i) {
      p_cur = sample_points_[i];
      length_ += (p_cur - p_prev).norm();
      p_prev = p_cur;
    }
    return length_;
  }

  double getMeanSpeed() {
    if (time_sum_ < 0) getTotalTime();
    if (length_ < 0) getLength();
    return length_ / time_sum_;
  }

  // Compute the integral of squared derivative (k is the order) along the trajectory
  double getIntegralCost(const int& k) {
    double cost = 0.0;
    if (time_sum_ < 0) getTotalTime();

    for (double ts = 0; ts < time_sum_; ts += 0.01) {
      Eigen::Vector3d um = evaluate(ts, k);
      cost += um.squaredNorm() * 0.01;
    }
    return cost;
  }

  void getMeanAndMaxDerivative(double& mean_d, double& max_d, const int& k) {
    mean_d = max_d = 0;
    int sample_num = 0;
    if (time_sum_ < 0) getTotalTime();
    for (double ts = 0; ts < time_sum_; ts += 0.01) {
      auto ds = evaluate(ts, k).norm();
      mean_d += ds;
      if (ds > max_d) max_d = ds;
      sample_num++;
    }
    mean_d /= double(sample_num);
  }

  // input : position of waypoints, start/end vel and acc, segment time
  // Pos: Nx3
  static void waypointsTraj(const Eigen::MatrixXd& positions, const Eigen::Vector3d& start_vel,
                            const Eigen::Vector3d& end_vel, const Eigen::Vector3d& start_acc,
                            const Eigen::Vector3d& end_acc, const Eigen::VectorXd& times,
                            PolynomialTraj& poly_traj);

private:
  vector<Polynomial> segments_;
  vector<double> times_;  // Time duration of each segment

  // Properties of traj
  double time_sum_;
  vector<Eigen::Vector3d> sample_points_;
  double length_;
};

PolynomialTraj fastLine4deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc,
                            double max_jerk);
PolynomialTraj fastLine3deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc);
}

#endif
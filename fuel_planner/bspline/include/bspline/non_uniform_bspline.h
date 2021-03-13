#ifndef _NON_UNIFORM_BSPLINE_H_
#define _NON_UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace fast_planner {
// An implementation of non-uniform B-spline with different dimensions
// It also represents uniform B-spline which is a special case of non-uniform
class NonUniformBspline {
public:
  NonUniformBspline();
  NonUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);
  ~NonUniformBspline();

  // initialize as an uniform B-spline
  void setUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);

  // get / set basic bspline info

  void setKnot(const Eigen::VectorXd& knot);
  Eigen::VectorXd getKnot();
  Eigen::MatrixXd getControlPoint();
  double getKnotSpan();
  void getTimeSpan(double& um, double& um_p);

  // compute position / derivative

  Eigen::VectorXd evaluateDeBoor(const double& u);   // use u \in [up, u_mp]
  Eigen::VectorXd evaluateDeBoorT(const double& t);  // use t \in [0, duration]
  void computeDerivatives(const int& k, vector<NonUniformBspline>& ders);
  NonUniformBspline getDerivative();
  void getBoundaryStates(const int& ks, const int& ke, vector<Eigen::Vector3d>& start,
                         vector<Eigen::Vector3d>& end);

  // 3D B-spline interpolation of points in point_set, with boundary vel&acc
  // constraints
  // input : (K+2) points with boundary vel/acc; ts
  // output: (K+6) control_pts
  static void parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                    const vector<Eigen::Vector3d>& start_end_derivative,
                                    const int& degree, Eigen::MatrixXd& ctrl_pts);

  // Check feasibility, adjust time
  void setPhysicalLimits(const double& vel, const double& acc);
  void lengthenTime(const double& ratio);
  double checkRatio();

  bool checkFeasibility(bool show = false);
  bool reallocateTime(bool show = false);

  // Evaluation
  double getTimeSum();
  double getLength(const double& res = 0.01);
  double getJerk();
  void getMeanAndMaxVel(double& mean_v, double& max_v);
  void getMeanAndMaxAcc(double& mean_a, double& max_a);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // control points for B-spline with different dimensions.
  // Each row represents one single control point
  // The dimension is determined by column number
  // e.g. B-spline with N points in 3D space -> Nx3 matrix
  Eigen::MatrixXd control_points_;

  int p_, n_, m_;      // p degree, n+1 control points, m = n+p+1
  Eigen::VectorXd u_;  // knots vector
  double knot_span_;   // knot span \delta t

  Eigen::MatrixXd getDerivativeControlPoints();

  double limit_vel_, limit_acc_, limit_ratio_;  // physical limits and time adjustment ratio
};
}  // namespace fast_planner
#endif
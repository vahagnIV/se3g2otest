//
// Created by vahagn on 31.03.22.
//

#ifndef G2OTEST_SRC_MONOCULAR_CAMERA_H_
#define G2OTEST_SRC_MONOCULAR_CAMERA_H_

#include <Eigen/Eigen>

class MonocularCamera {
 public:
  MonocularCamera(double fx, double fy, double cx, double cy);
  double Fx() const { return fx_; }
  double Fy() const { return fy_; }
  double Cx() const { return cx_; }
  double Cy() const { return cy_; }
  Eigen::Matrix<double, 2, 3> GetJacobian(const Eigen::Vector3d &point);
  Eigen::Vector2d Project(const Eigen::Vector3d &point) const;
 private:
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  Eigen::Matrix<double, 2, 2> f_jac;

};

#endif //G2OTEST_SRC_MONOCULAR_CAMERA_H_

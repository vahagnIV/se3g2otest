//
// Created by vahagn on 31.03.22.
//

#include "monocular_camera.h"

MonocularCamera::MonocularCamera(double fx, double fy, double cx, double cy) : fx_(fx),
                                                                               fy_(fy),
                                                                               cx_(cx),
                                                                               cy_(cy) {

  f_jac << fx, 0, 0, fy;
}

Eigen::Matrix<double, 2, 3> MonocularCamera::GetJacobian(const Eigen::Vector3d &point) {
  double z_inv = 1. / point.z();
  double z_inv2 = z_inv * z_inv;

  Eigen::Matrix<double, 2, 3> projection_jacobian;

  projection_jacobian << z_inv, 0, -point.x() * z_inv2,
      0, z_inv, -point.y() * z_inv2;

  return f_jac * projection_jacobian;
}

Eigen::Vector2d MonocularCamera::Project(const Eigen::Vector3d &point) const {
  double z_inv = 1. / point.z();
  return Eigen::Vector2d{point.x() * z_inv * Fx() + Cx(), point.y() * z_inv * Fy() + Cy()};
}

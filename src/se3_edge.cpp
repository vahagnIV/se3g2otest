//
// Created by vahagn on 31.03.22.
//

#include "se3_edge.h"

SE3Edge::SE3Edge(MonocularCamera *camera):camera_(camera) {

}

void SE3Edge::computeError() {
  auto vertex_position = dynamic_cast<g2o::VertexSE3Expmap *>(vertex(0));
  auto vertex_point = dynamic_cast<g2o::VertexPointXYZ *>(vertex(1));
  Eigen::Vector3d point_in_local_coordinates = vertex_position->estimate().map(vertex_point->estimate());
  error() = camera_->Project(point_in_local_coordinates) - measurement();
}

bool SE3Edge::read(std::istream &is) {
  return false;
}

bool SE3Edge::write(std::ostream &os) const {
  return false;
}

void SE3Edge::linearizeOplus() {
  auto vertex_position = dynamic_cast<g2o::VertexSE3Expmap *>(vertex(0));
  auto vertex_point = dynamic_cast<g2o::VertexPointXYZ *>(vertex(1));

  g2o::Vector3 point_in_local_coordinates = vertex_position->estimate().map(vertex_point->estimate());
  Eigen::Matrix<double, 2, 3> camera_jacobian = camera_->GetJacobian(point_in_local_coordinates);
  const double & x = point_in_local_coordinates.x();
  const double & y = point_in_local_coordinates.y();
  const double & z = point_in_local_coordinates.z();
  Eigen::Matrix<double, 3, 6> se3_jacobian;
  // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
  se3_jacobian << 0.f, z, -y, 1., 0., 0.,
      -z, 0.f, x, 0., 1., 0.,
      y, -x, 0.f, 0., 0., 1.;
  _jacobianOplusXi = camera_jacobian * se3_jacobian;
  _jacobianOplusXj = camera_jacobian * vertex_position->estimate().rotation().toRotationMatrix();
}

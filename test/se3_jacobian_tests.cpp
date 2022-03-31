//
// Created by vahagn on 31.03.22.
//

#include "se3_jacobian_tests.h"
#include "se3_edge.h"
#include "g2o/core/jacobian_workspace.h"
#include "test_utils.h"

namespace g2o {
template<typename EdgeType>
void evaluateJacobian(EdgeType &e, JacobianWorkspace &jacobianWorkspace,
                      g2o::JacobianWorkspace &numericJacobianWorkspace) {
  // calling the analytic Jacobian but writing to the numeric workspace
  e.BaseBinaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
                   typename EdgeType::VertexXiType,
                   typename EdgeType::VertexXjType>::linearizeOplus(numericJacobianWorkspace);
  // copy result into analytic workspace
  jacobianWorkspace = numericJacobianWorkspace;

  // compute the numeric Jacobian into the numericJacobianWorkspace workspace as
  // setup by the previous call
  e.BaseBinaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
                   typename EdgeType::VertexXiType,
                   typename EdgeType::VertexXjType>::linearizeOplus();

  // compare the two Jacobians
  for (int i = 0; i < 2; ++i) {
    number_t *n = numericJacobianWorkspace.workspaceForVertex(i);
    number_t *a = jacobianWorkspace.workspaceForVertex(i);
    int numElems = EdgeType::Dimension;
    if (i == 0)
      numElems *= EdgeType::VertexXiType::Dimension;
    else
      numElems *= EdgeType::VertexXjType::Dimension;
    for (int j = 0; j < numElems; ++j) {
//      std::cout <<"n[" << j << "] - a[" << j << "] = " << n[j] - a[j] << std::endl;
      EXPECT_NEAR(n[j], a[j], 1e-4);
    }
  }
}
}

SE3JacobianTests::SE3JacobianTests() : camera_(800, 800, 320, 240), edge_(&camera_) {

  edge_.setVertex(0, &vertex_position);
  edge_.setVertex(1, &vertex_point);
  edge_.setInformation(Eigen::Matrix2d::Identity());

  numeric_jacobian_workspace_.updateSize(&edge_);
  numeric_jacobian_workspace_.allocate();

}

TEST_F(SE3JacobianTests, JacobianIsComputedCorrectly) {

    Eigen::Vector3d local_point = GetRandomPoint();
    Eigen::Matrix3d R = GetRandomRotationMatrix();
    Eigen::Vector3d T = GetRandomPosition();
    g2o::SE3Quat quat(R,T);

    Eigen::Vector3d world_point = quat.inverse().map(local_point);
    vertex_point.setEstimate(world_point);
    vertex_position.setEstimate(quat);
    edge_.setMeasurement(camera_.Project(local_point));


    g2o::evaluateJacobian(edge_, jacobian_workspace_, numeric_jacobian_workspace_);

}
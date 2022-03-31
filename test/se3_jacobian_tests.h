//
// Created by vahagn on 31.03.22.
//

#ifndef G2OTEST_TEST_SE_3_JACOBIAN_TESTS_H_
#define G2OTEST_TEST_SE_3_JACOBIAN_TESTS_H_
#include <gtest/gtest.h>
#include "monocular_camera.h"
#include "se3_edge.h"
#include <g2o/core/jacobian_workspace.h>
class SE3JacobianTests : public testing::Test {
 public:
  SE3JacobianTests();
 protected:
  MonocularCamera camera_;
  SE3Edge edge_;
  g2o::JacobianWorkspace jacobian_workspace_;
  g2o::JacobianWorkspace numeric_jacobian_workspace_;
  g2o::VertexPointXYZ vertex_point;
  g2o::VertexSE3Expmap vertex_position;

};

#endif //G2OTEST_TEST_SE_3_JACOBIAN_TESTS_H_

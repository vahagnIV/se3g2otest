//
// Created by vahagn on 31.03.22.
//

#ifndef G2OTEST_SRC_SE_3_EDGE_H_
#define G2OTEST_SRC_SE_3_EDGE_H_
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/core/base_binary_edge.h>
#include "monocular_camera.h"
class SE3Edge : public g2o::BaseBinaryEdge<2, Eigen::Vector2d,
                                           g2o::VertexSE3Expmap,
                                           g2o::VertexPointXYZ> {
 public:
  SE3Edge(MonocularCamera *camera);
  void computeError() override;
  bool read(std::istream &is) override;
  bool write(std::ostream &os) const override;
  void linearizeOplus() override;
 private:
  MonocularCamera *camera_;

};

#endif //G2OTEST_SRC_SE_3_EDGE_H_

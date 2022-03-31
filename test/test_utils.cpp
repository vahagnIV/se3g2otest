//
// Created by vahagn on 31.03.22.
//

#include "test_utils.h"

Eigen::Matrix<double, 3, 3> GetRotationMatrixRollPitchYaw(double alpha, double beta, double gamma) {
  Eigen::Matrix<double, 3, 3> X, Y, Z;
  X << 1, 0, 0,
      0, std::cos(alpha), -std::sin(alpha),
      0, std::sin(alpha), std::cos(alpha);
  Y << std::cos(beta), 0, -std::sin(beta),
      0, 1, 0,
      std::sin(beta), 0, std::cos(beta);
  Z << std::cos(gamma), -std::sin(gamma), 0,
      std::sin(gamma), std::cos(gamma), 0,
      0, 0, 1;

  return X * Y * Z;
}
double GetRandomDouble(double fMin, double fMax) {
  double f = (double) rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}
Eigen::Matrix<double, 3, 3> GetRandomRotationMatrix() {
  return GetRotationMatrixRollPitchYaw(GetRandomDouble(-M_PI / 2, M_PI / 2),
                                       GetRandomDouble(-M_PI / 2, M_PI / 2),
                                       GetRandomDouble(-M_PI / 2, M_PI / 2));
}

Eigen::Vector3d GetRandomPosition() {
  return Eigen::Vector3d{GetRandomDouble(-10, 10), GetRandomDouble(-10, 10), GetRandomDouble(-10, 10)};
}

Eigen::Vector3d GetRandomPoint() {
  return Eigen::Vector3d{GetRandomDouble(-10, 10), GetRandomDouble(-10, 10), GetRandomDouble(1, 30)};
}

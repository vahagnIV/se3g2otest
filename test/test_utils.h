//
// Created by vahagn on 31.03.22.
//

#ifndef G2OTEST_TEST_TEST_UTILS_H_
#define G2OTEST_TEST_TEST_UTILS_H_
#include <Eigen/Eigen>

Eigen::Matrix<double, 3, 3> GetRotationMatrixRollPitchYaw(double alpha, double beta, double gamma);

double GetRandomDouble(double fMin, double fMax);

Eigen::Matrix<double, 3, 3> GetRandomRotationMatrix();
Eigen::Vector3d GetRandomPoint();
Eigen::Vector3d GetRandomPosition();


#endif //G2OTEST_TEST_TEST_UTILS_H_

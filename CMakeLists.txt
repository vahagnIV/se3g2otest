cmake_minimum_required(VERSION 3.21)
project(g2oTest)

set(CMAKE_CXX_STANDARD 14)
find_package(g2o)
find_package(Eigen3)

add_subdirectory(src)
add_subdirectory(test)



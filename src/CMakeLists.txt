find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(${CMAKE_CURRENT_LIST_DIR}
        g2o
        ${CHOLMOD_INCLUDE_DIR})

add_library(testg2o se3_edge.cpp monocular_camera.cpp)
target_include_directories(testg2o PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(testg2o g2o::core
        g2o::solver_cholmod
        Eigen3::Eigen
        g2o::types_sba
        g2o::types_sim3)
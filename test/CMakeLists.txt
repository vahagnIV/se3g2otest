configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test/googletest-download)
if (result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif ()
find_package(GTest)
find_package(OpenCV)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test/googletest-download)
if (result)
    message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif ()

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/test/googletest-src
        ${CMAKE_BINARY_DIR}/test/googletest-build
        EXCLUDE_FROM_ALL)

set(ALL_TEST_SOURCE_FILES
        se3_jacobian_tests.cpp
        test_utils.cpp)
add_executable(unit_tests ${ALL_TEST_SOURCE_FILES})
target_link_libraries(unit_tests gmock gmock_main
        testg2o
        g2o::core
        g2o::solver_cholmod
        Eigen3::Eigen
        g2o::types_sba
        g2o::types_sim3)

gtest_add_tests(TARGET unit_tests
        SOURCES ${ALL_TEST_SOURCE_FILES}
        TEST_SUFFIX Tests
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

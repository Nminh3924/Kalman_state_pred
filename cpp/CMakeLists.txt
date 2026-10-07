cmake_minimum_required(VERSION 3.18)
project(kalman_tracking CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# Optimized by default, so that the run times the program reports mean something.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

add_library(kalman
    src/ctrv_ekf.cpp
    src/ctrv_ukf.cpp
    src/cv_kalman_filter.cpp
    src/forecast.cpp
    src/io.cpp
    src/kalman_filter.cpp
    src/run.cpp
    src/unscented.cpp)
target_include_directories(kalman PUBLIC include)
target_link_libraries(kalman PUBLIC Eigen3::Eigen)

add_executable(main src/main.cpp)
target_link_libraries(main PRIVATE kalman)

enable_testing()
add_executable(tests
    tests/test_constant_velocity.cpp
    tests/test_ctrv.cpp
    tests/test_ctrv_filters.cpp
    tests/test_forecast.cpp
    tests/test_io.cpp
    tests/test_kalman_filter.cpp
    tests/test_run.cpp
    tests/test_unscented.cpp)
target_link_libraries(tests PRIVATE kalman GTest::gtest_main)
include(GoogleTest)
# List the tests when ctest runs, not in a build step: the build step breaks
# when the path of the project contains a character such as "&".
gtest_discover_tests(tests DISCOVERY_MODE PRE_TEST)

cmake_minimum_required(VERSION 2.8.9)
project(optimal_visual_servoing)
set (CMAKE_CXX_STANDARD 11)

find_package(Ceres REQUIRED)

add_executable(optimization_problem src/OptimizationProblem.cc)
target_link_libraries(optimization_problem ceres)

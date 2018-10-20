cmake_minimum_required(VERSION 2.8.9)
project(optimal_visual_servoing)
set (CMAKE_CXX_STANDARD 11)

include_directories(
	include/
	Eigen	
	${OpenCV_INCLUDE_DIRS}
)

find_package(Ceres REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(optimization_problem src/OptimizationProblem.cc src/CircleOfInterest.cc src/ArucoTagsDetection.cc src/DynamicWindow.cc)
target_link_libraries(optimization_problem ceres ${OpenCV_LIBRARIES})

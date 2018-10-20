cmake_minimum_required(VERSION 2.8.9)
project(optimal_visual_servoing)
set (CMAKE_CXX_STANDARD 11)

find_package (catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs)
	
catkin_package(
#	INCLUDE_DIRS include
	LIBRARIES optimal_visual_servoing
	CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros
)

include_directories(
	include/
	Eigen
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${ROS_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package (Ceres REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (OpenCV REQUIRED)
find_package (PCL REQUIRED COMPONENTS common io)


add_executable(optimal_visual_servoing src/OptimizationProblem.cc src/CircleOfInterest.cc src/ArucoTagsDetection.cc src/DynamicWindowSampler.cc src/Wrapper.cc src/LaserSegmentation.cc)
target_link_libraries(optimal_visual_servoing ceres ${OpenCV_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} pcl_search pcl_segmentation)

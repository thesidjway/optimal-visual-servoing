cmake_minimum_required(VERSION 2.8.9)
project(optimal_visual_servoing)
set (CMAKE_CXX_STANDARD 11)

find_package (catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  cv_bridge
  laser_geometry
  moveit_ros_planning_interface
  std_msgs)

catkin_package(
#	INCLUDE_DIRS include
	LIBRARIES optimal_visual_servoing
	CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros cv_bridge laser_geometry moveit_ros_planning_interface
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


add_executable(optimal_visual_servoing src/Optimization.cc src/ArucoTagsDetection.cc src/DynamicWindowSampler.cc src/Wrapper.cc src/ClusterExtractor.cc src/RobotMoveIt.cc)
target_link_libraries(optimal_visual_servoing ceres ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} pcl_search pcl_segmentation yaml-cpp)

add_executable(calibration calibration/charuco_calib.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(tag_trajectory_generator src/TagTrajectoryGenerator.cc)
target_link_libraries(tag_trajectory_generator ${catkin_LIBRARIES} yaml-cpp)

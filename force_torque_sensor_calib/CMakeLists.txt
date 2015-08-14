cmake_minimum_required(VERSION 2.8.3)
project(force_torque_sensor_calib)

find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp sensor_msgs tf geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface tf_conversions eigen_conversions)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  DEPENDS eigen
  CATKIN_DEPENDS roscpp sensor_msgs tf geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface eigen tf_conversions eigen_conversions
  INCLUDE_DIRS include 
  LIBRARIES 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(ft_calib_node src/ft_calib.cpp src/ft_calib_node.cpp)
target_link_libraries(ft_calib_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ft_calib_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config include launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

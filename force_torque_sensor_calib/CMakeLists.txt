cmake_minimum_required(VERSION 2.8.3)
project(force_torque_sensor_calib)

find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp sensor_msgs tf geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface tf_conversions eigen_conversions)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  DEPENDS eigen
  CATKIN_DEPENDS roscpp sensor_msgs tf geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface eigen tf_conversions eigen_conversions
  INCLUDE_DIRS include
  LIBRARIES
)

add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB_RECURSE HEADERS include/${PROJECT_NAME}/*.h)

add_executable(ft_calib_node src/ft_calib.cpp src/ft_calib_node.cpp ${HEADERS})
target_link_libraries(ft_calib_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ft_calib_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config include launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulator_support)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
        )



catkin_package(
  LIBRARIES dual_sia5d_support
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(dualManipulator src/dualManipulator.cpp)
target_link_libraries(dualManipulator ${catkin_LIBRARIES})
add_dependencies(dualManipulator dual_manipulator_support_gencpp)



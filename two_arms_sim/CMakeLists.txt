cmake_minimum_required(VERSION 2.8.3)
project(two_arms_sim)
                
#------------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rmf_msgs
  actionlib
  control_msgs
  trajectory_msgs
  mongodb_store
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  moveit_ros_move_group
)

find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    actionlib
    control_msgs
    trajectory_msgs
    mongodb_store
)
include_directories(include ${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
#------------------------------------------------------------------------------
add_executable(two_arms_sim src/test.cpp src/arm_traj_controller.cpp)
set_property(TARGET two_arms_sim PROPERTY CXX_STANDARD 17)
target_link_libraries(two_arms_sim PRIVATE
  ${catkin_LIBRARIES}
)
#------------------------------------------------------------------------------
add_executable(moveit_example src/moveit_example.cpp)
target_link_libraries(moveit_example ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS moveit_example DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#------------------------------------------------------------------------------
target_include_directories(two_arms_sim PRIVATE
  ${catkin_INCLUDE_DIRS}
)
install(TARGETS two_arms_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



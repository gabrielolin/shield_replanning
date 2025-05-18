cmake_minimum_required(VERSION 3.0.2)
project(replanning)

add_compile_options(-std=c++17)

set(MUJOCO_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third_party/mujoco-2.3.2/include")
set(MUJOCO_LIB_DIR "${CMAKE_SOURCE_DIR}/third_party/mujoco-2.3.2/lib")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  leatherman
  moveit_core
  moveit_ros_planning_interface
  roslib
  smpl_ros
  shield_planner
  sbpl_collision_checking
  sbpl_kdl_robot_model
  # parallel_search)
)
find_package(smpl REQUIRED)
catkin_package(
  CATKIN_DEPENDS roscpp actionlib control_msgs shield_planner shield_planner_msgs 
)

# Path to Ruckig inside your conda env
include_directories(
  ${catkin_INCLUDE_DIRS}
  $ENV{CONDA_PREFIX}/include
  ${MUJOCO_INCLUDE_DIR}
)

link_directories(
  $ENV{CONDA_PREFIX}/lib
)

link_directories(${MUJOCO_LIB_DIR})
add_executable(ruckig_test src/ruckig_test.cpp)

target_link_libraries(ruckig_test
  ${catkin_LIBRARIES}
  ruckig
  mujoco
)
add_executable(fake_perception_node src/fake_perception_node.cpp)
target_link_libraries(fake_perception_node ${catkin_LIBRARIES})


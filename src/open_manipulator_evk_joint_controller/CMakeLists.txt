cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_evk_joint_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 control_msgs
 trajectory_msgs
 actionlib
)

catkin_package(
 CATKIN_DEPENDS
)

include_directories(
 ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_executable(
  open_manipulator_evk_joint_controller
  src/open_manipulator_evk_joint_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

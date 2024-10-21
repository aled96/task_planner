cmake_minimum_required(VERSION 3.0.2)
project(task_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  SolveTask.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_generation message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  src/task_planner/TaskPlanner.py
  scripts/TaskPlannerNode.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

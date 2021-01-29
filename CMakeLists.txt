cmake_minimum_required(VERSION 3.10)
project(tiago_webots_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib  
  behaviortree_cpp_v3
  iris_lama_ros
  gmapping
  move_base
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  teb_local_planner
  tf
  webots_ros
  xmlrpcpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib  
    behaviortree_cpp_v3
    iris_lama_ros
    gmapping
    move_base
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
    teb_local_planner
    tf
    webots_ros
    xmlrpcpp
)

#############
##  Build  ##
#############
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/robot_task.cc)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(robot_task_node src/robot_task_node.cc)
add_dependencies(robot_task_node ${PROJECT_NAME})
target_link_libraries(robot_task_node ${PROJECT_NAME})


#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
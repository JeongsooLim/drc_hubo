cmake_minimum_required(VERSION 2.8.3)
project(drc_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  gazebo_ros
  std_msgs
  message_generation
  tf
)

add_message_files(FILES DRC_HEAD_CMD.msg)
generate_messages(DEPENDENCIES std_msgs)


# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging)

add_library(drc_plugin src/drc_plugin.cpp)
add_library(head_plugin src/head_plugin.cpp)
add_dependencies(head_plugin head_plugin_generate_messages_cpp)
target_link_libraries(drc_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(head_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    std_msgs
)

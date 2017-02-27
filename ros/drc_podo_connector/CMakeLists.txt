cmake_minimum_required(VERSION 2.8.3)
project(drc_podo_connector)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tf
  nav_msgs
)

add_message_files(
  FILES
  DRC_HEAD_CMD.msg
  SendPos.msg
)
generate_messages(DEPENDENCIES std_msgs)




include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(podo_connector src/podo_connector.cpp)
add_dependencies(podo_connector drc_podo_connector_generate_messages_cpp)
target_link_libraries(podo_connector ${catkin_LIBRARIES})


catkin_package(
  DEPENDS
    roscpp
    std_msgs
)

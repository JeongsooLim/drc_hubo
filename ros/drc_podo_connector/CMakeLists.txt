cmake_minimum_required(VERSION 2.8.3)
project(drc_podo_connector)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

add_message_files(FILES DRC_HEAD_CMD.msg)
add_message_files(FILES SendPos.msg)
generate_messages(DEPENDENCIES std_msgs)



SET(LIBRARIES
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(podo_connector src/podo_connector.cpp)
target_link_libraries(podo_connector ${LIBRARIES})


catkin_package(
  DEPENDS
    roscpp
)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(gip_optimal_navigation)

find_package(catkin REQUIRED COMPONENTS
  gazebo
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  obstacleMsg.msg
  obstacleMsg_vector.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp message_runtime rospy
   DEPENDS eigen)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_library(hello_world SHARED hello_world.cc)
#target_link_libraries(hello_world ${GAZEBO_LIBRARIES})

add_library(model_push SHARED src/model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

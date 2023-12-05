cmake_minimum_required(VERSION 3.0.2)
project(serial_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  genmsg
)

add_message_files(
  DIRECTORY
  msg
  FILES
  Num.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
  FILES
  AddTwoInts.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(listener src/listener.cpp)
add_executable(dengar src/dengar.cpp)
add_executable(autoTanam src/autoTanam.cpp)

target_link_libraries(autoTanam ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})
target_link_libraries(dengar ${catkin_LIBRARIES})

# add_dependencies(listener serial_ros_generate_messages_cpp)
# add_dependencies(dengar serial_ros_generate_messages_cpp)
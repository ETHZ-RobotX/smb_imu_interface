cmake_minimum_required(VERSION 3.0.2)

project(versavis_adis16448_receiver)

# Set compile flags (-Ofast actually makes a big difference over -O3 here (maybe 20% faster)
set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  rosserial_arduino
  rosserial_client
  rosserial_python
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  ImuMicro.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

include_directories(
	include
    ${catkin_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############

add_executable(versavis_imu_receiver
   src/versavis_imu_receiver.cpp
)
add_dependencies(versavis_imu_receiver ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(versavis_imu_receiver ${catkin_LIBRARIES})

############
## EXPORT ##
############
# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

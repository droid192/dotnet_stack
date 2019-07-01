OPTION(BUILD_DOTNECT_PLATFORM "Build platform node")
set(BUILD_DOTNECT_PLATFORM ON)
if (BUILD_DOTNECT_PLATFORM)

cmake_minimum_required(VERSION 2.8.3)
project(dotnect_platform CXX)

find_package(catkin REQUIRED COMPONENTS
  dotnect_custom_msgs
  cv_bridge
  image_transport
  roscpp
  std_msgs
  tf
  visualization_msgs
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(CATKIN_DEPENDS
  dotnect_custom_msgs
  cv_bridge
  image_transport
  message_runtime
  roscpp
  std_msgs
  tf
  visualization_msgs
)

add_executable(	dotnect_platform
  src/dotnect_platform.cpp
  src/moving_average.cpp
  src/val3_fstream_socket.cpp
  src/data_container.cpp
  src/tracking_class.cpp
)

add_dependencies(dotnect_platform
# ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(dotnect_platform
  ${catkin_LIBRARIES}
  ${Boost_LIBRARY}
  ${OpenCV_LIBS}
)

install(TARGETS dotnect_platform
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# test node
add_executable(test_ui_signals
  test/test_ui_signals.cpp
)

target_link_libraries(test_ui_signals
  ${catkin_LIBRARIES}
)

install(TARGETS test_ui_signals
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()

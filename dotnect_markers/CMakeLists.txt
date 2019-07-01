OPTION(BUILD_DOTNECT_MARKERS "Build marker node")
set(BUILD_DOTNECT_MARKERS ON)
if (BUILD_DOTNECT_MARKERS)

cmake_minimum_required(VERSION 2.8.3)
project(dotnect_markers CXX)

find_package(catkin REQUIRED COMPONENTS
  dotnect_custom_msgs
  interactive_markers
  roscpp
  tf
  std_msgs
  visualization_msgs
)

find_package(Boost REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(CATKIN_DEPENDS
  dotnect_custom_msgs
  interactive_markers
  roscpp
  tf
  std_msgs
  visualization_msgs
)

add_executable(dotnect_markers
  src/dotnect_markers.cpp
)

# Let dotnect_custom_msgs create its header before start linking
add_dependencies(dotnect_markers
  dotnect_custom_msgs_gencpp
)

target_link_libraries(dotnect_markers
  ${catkin_LIBRARIES}
)

install(TARGETS dotnect_markers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()

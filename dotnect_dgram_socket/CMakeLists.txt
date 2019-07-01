OPTION(BUILD_DOTNECT_DOTNECT_DGRAM_SOCKET "Build dgram socket node")
set(BUILD_DOTNECT_DOTNECT_DGRAM_SOCKET ON)
if (BUILD_DOTNECT_DOTNECT_DGRAM_SOCKET)

cmake_minimum_required(VERSION 2.8.3)
project(dotnect_dgram_socket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  std_msgs
)

find_package(Boost REQUIRED)

include_directories(
  include
  test
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(CATKIN_DEPENDS
  std_srvs
  std_msgs
)

add_executable(dotnect_dgram_socket
  src/dotnect_dgram_socket.cpp
)

target_link_libraries(dotnect_dgram_socket
  ${catkin_LIBRARIES}
  ${Boost_LIBRARY}
)

install(TARGETS dotnect_dgram_socket
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# test node
add_executable(test_dotnect_dgram_socket
  test/test_dotnect_dgram_socket.cpp
)

target_link_libraries(test_dotnect_dgram_socket
  ${catkin_LIBRARIES}
)

install(TARGETS test_dotnect_dgram_socket
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()

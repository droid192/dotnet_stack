OPTION(BUILD_DOTNECT_RVIZ_PLUGIN "Build dotnect_rviz_plugin")
set(BUILD_DOTNECT_RVIZ_PLUGIN OFF)
if (BUILD_DOTNECT_RVIZ_PLUGIN)

cmake_minimum_required(VERSION 2.8.3)
project(dotnect_rviz_plugin)

# ros packages
find_package(catkin REQUIRED COMPONENTS
  rviz
)

catkin_package(
   # INCLUDE_DIRS
   # LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS rviz
   # DEPENDS
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

# Bundle headers .h
set(HEADER_FILES
  include/drive_widget.h
  include/teleop_panel.h
)

# Automatically enable Qt's Meta-Object Compiler moc generation
set(CMAKE_AUTOMOC ON)

# Avoid keyword definition to avaid conflicts with boost or xapian etc
# e.g. http://muddyazian.blogspot.de/2012/04/getting-qt-app-working-with-boost-using.html
add_definitions(-DQT_NO_KEYWORDS)

# Use same QT version as rviz and specify headers run through MOC.
# MOC generated .cpp files are included automatically as headers.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  qt4_wrap_cpp(HEADER_FILES_MOCS ${HEADER_FILES})
  #qt4_wrap_ui(rqt_image_view_UIS_H ${rqt_image_view_UIS})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  qt5_wrap_cpp(HEADER_FILES_MOCS ${HEADER_FILES})
  #qt5_wrap_ui(rqt_image_view_UIS_H ${rqt_image_view_UIS})
endif()

# Bundle source .cpp including the mocs
set(SOURCE_FILES
src/drive_widget.cpp
src/teleop_panel.cpp
${HEADER_FILES_MOCS}
)

# A rviz plugin is a shared library named ${PROJECT_NAME}.so
add_library(${PROJECT_NAME} ${SOURCE_FILES})

# Link executable with Qt libraries
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Install rules
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

install(PROGRAMS scripts/test_joints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()

cmake_minimum_required(VERSION 2.8.3)
project(annotate)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
geometry_msgs
interactive_markers
pcl_conversions
pcl_ros
roscpp
rviz
tf
visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS include
# LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS
geometry_msgs
interactive_markers
pcl_conversions
pcl_ros
roscpp
rviz
tf
visualization_msgs
)

###########
## Build ##
###########
include_directories(
include
${catkin_INCLUDE_DIRS}
${rviz_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets Gui)
## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt5::Widgets Qt5::Gui)
add_definitions(-DQT_NO_KEYWORDS)

# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp
# target_link_libraries(${PROJECT_NAME}_node ${QT_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

add_library(${PROJECT_NAME}
src/${PROJECT_NAME}_display.cpp
src/${PROJECT_NAME}_tool.cpp
src/annotation_marker.cpp
src/file_dialog_property.cpp
src/shortcut_property.cpp
include/${PROJECT_NAME}/${PROJECT_NAME}_display.h
include/${PROJECT_NAME}/${PROJECT_NAME}_tool.h
include/${PROJECT_NAME}/file_dialog_property.h
include/${PROJECT_NAME}/shortcut_property.h
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

#############
## Install ##
#############
#install(TARGETS ${PROJECT_NAME}_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

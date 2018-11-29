cmake_minimum_required(VERSION 2.8.3)
project(drive_ros_image_recognition)

#SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_BUILD_TYPE Debug)

# enable to compile with OpenCV window displays
add_definitions(-DDRAW_DEBUG)
# enable to compile with publishers of internally processed images
#add_definitions(-DPUBLISH_DEBUG)
# enable to use World2Cam homography
#add_definitions(-DUSE_WORLD2CAM_HOMOGRAPHY)
# enable publishing of detected points (for rviz display)
#add_definitions(-DPUBLISH_WORLD_POINTS)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    image_geometry
    roscpp
    sensor_msgs
    drive_ros_msgs
    nodelet
)

#include Boost for lane line processing
find_package(Boost REQUIRED)

#generate_dynamic_reconfigure_options(
#)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport image_geometry roscpp sensor_msgs drive_ros_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )


################################################################################
# road detection node (LMS port)
################################################################################

add_executable(image_processing_node
    src/image_processing.cpp
    src/image_processing_node.cpp
    )

add_dependencies(image_processing_node
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(image_processing_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    )

install(TARGETS image_processing_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

################################################################################
# road detection nodelet (LMS port)
################################################################################

add_library(image_processing_nodelet
    src/image_processing.cpp
    )

add_dependencies(image_processing_nodelet
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(image_processing_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    )

install(TARGETS image_processing_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

################################################################################
# Warp image node
################################################################################
add_executable(warp_image_node
    src/warp_image_node.cpp
    src/warp_image.cpp
    )

add_dependencies(warp_image_node
    ${drive_ros_msgs_generate_messages_cpp}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(warp_image_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

install(TARGETS warp_image_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

################################################################################
# Warp image nodelet
################################################################################
add_library(warp_image_nodelet
    src/warp_image.cpp
    )

add_dependencies(warp_image_nodelet
    ${drive_ros_msgs_generate_messages_cpp}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(warp_image_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

install(TARGETS warp_image_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

################################################################################
# Install other files
################################################################################
 install(PROGRAMS
   scripts/image_processing_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(FILES nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
    )

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
    )

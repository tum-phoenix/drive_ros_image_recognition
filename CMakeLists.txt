cmake_minimum_required(VERSION 2.8.3)
project(drive_ros_image_recognition)

#SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_BUILD_TYPE Debug)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

################################################################################
# Line recognition node
################################################################################
add_executable(line_recognition_node
    src/line_recognition_node.cpp
    src/line_recognition.cpp
    src/edge_point.cpp
#    src/line_point.cpp
    )

add_dependencies(line_recognition_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(line_recognition_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

install(TARGETS line_recognition_node
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

add_dependencies(warp_image_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
# Image publisher node
################################################################################
add_executable(image_publisher_node
    src/image_publisher_node.cpp
    )

add_dependencies(image_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(image_publisher_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

install(TARGETS image_publisher_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

################################################################################
# Install other files
################################################################################
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
    )

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.yaml"
    )


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

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    image_geometry
    roscpp
    sensor_msgs
    std_msgs
    drive_ros_msgs
    dynamic_reconfigure
    geometry_msgs
    message_generation
    nodelet
    tf
    message_filters
)

#include Boost for lane line processing
find_package(Boost REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/LineDetection.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs drive_ros_msgs
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

add_executable(road_detection_node
    src/road_detection.cpp
    src/road_detection_node.cpp
    )

add_dependencies(road_detection_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(road_detection_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    )

install(TARGETS road_detection_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(road_detection_node_test launch/road_detection.test
        test/common_image_operations_test.cpp
        test/road_detection_test.cpp
        test/geometry_common_test.cpp
        )
    target_link_libraries(road_detection_node_test ${catkin_LIBRARIES})
endif()


################################################################################
# road detection nodelet (LMS port)
################################################################################

add_library(road_detection_nodelet
    src/road_detection.cpp
    )

add_dependencies(road_detection_nodelet ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(road_detection_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    )

install(TARGETS road_detection_nodelet
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

add_dependencies(warp_image_node ${drive_ros_msgs_generate_messages_cpp} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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

add_dependencies(warp_image_nodelet ${drive_ros_msgs_generate_messages_cpp} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
# Street crossing detection nodelet (LMS port)
################################################################################

add_library(street_crossing_nodelet
    src/street_crossing.cpp
    )

add_dependencies(street_crossing_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(street_crossing_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    )

install(TARGETS street_crossing_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

################################################################################
# Street crossing detection node (LMS port)
################################################################################

add_executable(street_crossing_node
    src/street_crossing.cpp
    src/street_crossing_node.cpp
    )

add_dependencies(street_crossing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(street_crossing_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    )

install(TARGETS street_crossing_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

################################################################################
# Install other files
################################################################################
install(FILES nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

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

cmake_minimum_required(VERSION 2.8.3)
project(blob_tracking)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  message_filters
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_python_setup()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(/usr/local/include)

find_path(RGBD_UTILS_INCLUDE_DIR rgbd_utils/rgbd_subscriber.hpp)
find_library(RGBD_UTILS_LIBRARY NAMES rgbd_utils)

include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} /usr/local/include)
link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib)
add_definitions(${PCL_DEFINITIONS})



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blob_tracking
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry image_transport message_filters pcl_conversions roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   scripts/blob_tracking
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

add_executable(blob_tracking_2 src/blob_tracking.cpp)
target_link_libraries(blob_tracking_2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} rgbd_utils )

add_executable(points_transformer src/points_transformer.cpp)
target_link_libraries(points_transformer ${catkin_LIBRARIES} )

cmake_minimum_required(VERSION 2.8)
project(mapping2d_orbslam)

find_package(catkin REQUIRED nav_msgs openslam_gmapping roscpp rostest tf rosbag_storage)

find_package(Boost REQUIRED signals)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(src)

catkin_package()

add_executable(mapping src/slam_gmapping.cpp src/main.cpp)
target_link_libraries(mapping ${Boost_LIBRARIES} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(mapping ${catkin_EXPORTED_TARGETS})
endif()

add_library(mapping_nodelet src/slam_gmapping.cpp src/nodelet.cpp)
target_link_libraries(mapping_nodelet ${catkin_LIBRARIES})


install(TARGETS mapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

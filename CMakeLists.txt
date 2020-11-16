
cmake_minimum_required(VERSION 2.8.3)

project(pointcloud_registration)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_conversions)
find_package(PCL REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# message(FATAL_ERROR ${CMAKE_CURRENT_SOURCE_DIR}/include)

catkin_package()

add_executable(pointcloud_registration_node src/pointcloud_registration_node.cpp)
target_link_libraries(pointcloud_registration_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS pointcloud_registration_node
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

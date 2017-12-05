cmake_minimum_required(VERSION 2.8.3)

# c++14 support by doing this
# don't rely on compiler-specific extensions
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-std=c++14)

project(transform_imu)

find_package(Boost REQUIRED)

find_package(
	catkin REQUIRED
	COMPONENTS
		roscpp
		sensor_msgs
		tf2
		tf2_ros
		tf2_geometry_msgs
)

catkin_package()

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(transform_imu_node src/transform_imu_node.cpp)
target_link_libraries(transform_imu_node ${catkin_LIBRARIES})
add_dependencies(transform_imu_node ${catkin_EXPORTED_TARGETS})

install(TARGETS transform_imu_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

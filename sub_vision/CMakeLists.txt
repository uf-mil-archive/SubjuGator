# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sub_vision)

set(CMAKE_XX_FLAGS "-std=c++11")


find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	cv_bridge
)


catkin_package(
	LIBRARIES vision_tools
	CATKIN_DEPENDS std_msgs
	CATKIN_DEPENDS geometry_msgs
	DEPENDS include
)

set(YOUR_LIB_SOURCES
	src/vision_tools.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(recovery_vision nodes/recovery_vision.cpp)
add_library(vision_tools ${YOUR_LIB_SOURCES})
target_link_libraries(recovery_vision ${catkin_LIBRARIES} vision_tools)

add_dependencies(recovery_vision sub_vision_generate_messages_cpp)
add_dependencies(recovery_vision src/vision_tools.cpp)

add_definitions(-std=c++11)

install(TARGETS vision_tools
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"

)
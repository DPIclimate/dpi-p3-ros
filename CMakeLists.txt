cmake_minimum_required(VERSION 3.0.2)

project(dpi-p3-ros)

# Source files
set(SRC_DIR
	src/main.cpp
	src/controller.cpp
)

# Header files
set(INC_DIR
	include/main.hpp
	include/controller.hpp
)

# Set output directories
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Include the following packages from catkin
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	geometry_msgs
)

# Keyboard input/output
find_package(Curses REQUIRED)

# Messages for catkin (ROS)
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

# Catkin package setup with dependencies
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS geometry_msgs roscpp std_msgs
	CATKIN_DEPENDS message_runtime
	DEPENDS system_lib
)

# Add libraries
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CURSES_INCLUDE_DIR} 
	${catkin_INCLUDE_DIRS})

# Add an executable
add_executable(${PROJECT_NAME} ${SRC_DIR} ${INC_DIR})

# Link libraries to executable
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CURSES_LIBRARY})


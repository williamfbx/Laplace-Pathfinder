cmake_minimum_required(VERSION 3.8)
project(laplace_pathfinder)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

set(TORCH_LIBRARIES_CLEAN "")
foreach(TORCH_LIB ${TORCH_LIBRARIES})
	if(NOT TORCH_LIB MATCHES "NOTFOUND")
		list(APPEND TORCH_LIBRARIES_CLEAN ${TORCH_LIB})
	endif()
endforeach()

add_executable(robot_nav_controller
	src/robot_nav_controller_main.cpp
	src/robot_nav_controller_node.cpp
)

target_include_directories(robot_nav_controller PUBLIC
	include
)

ament_target_dependencies(robot_nav_controller
	geometry_msgs
	nav_msgs
	rclcpp
)

add_executable(perturbation
	src/perturbation_main.cpp
	src/perturbation_node.cpp
	src/utils.cpp
)

target_include_directories(perturbation PUBLIC
	include
)

ament_target_dependencies(perturbation
	nav_msgs
	rclcpp
)

target_link_libraries(perturbation ${TORCH_LIBRARIES_CLEAN})
message(STATUS "libtorch found. Enabling torch NN inference in perturbation node.")

add_executable(robot_nav_planner
	src/robot_nav_planner_main.cpp
	src/robot_nav_planner_node.cpp
	src/utils.cpp
)

target_include_directories(robot_nav_planner PUBLIC
	include
)

ament_target_dependencies(robot_nav_planner
	geometry_msgs
	nav_msgs
	rclcpp
)

install(
	DIRECTORY launch config maps models
	DESTINATION share/${PROJECT_NAME}
)

install(
	DIRECTORY include/
	DESTINATION include
)

install(
	TARGETS robot_nav_controller robot_nav_planner perturbation
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()

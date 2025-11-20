cmake_minimum_required(VERSION 3.15)

if(POLICY CMP0144)
	cmake_policy(SET CMP0144 NEW)
	set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

# Prefer modern Python lookup modules over removed legacy ones
if(POLICY CMP0148)
	# rosidl currently relies on FindPythonInterp; keep OLD to avoid CMake
	# looking only for the newer FindPython3 module.
	cmake_policy(SET CMP0148 OLD)
endif()

if(POLICY CMP0167)
	cmake_policy(SET CMP0167 NEW)
	set(CMAKE_POLICY_DEFAULT_CMP0167 NEW)
endif()

project(height_mapping_ros2)

# Default to Release builds if no type is specified (single-config generators)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------- #
# --- Find dependencies --- #
# ------------------------- #

# Find ament packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find message packages
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Find grid_map packages
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)

# CV bridge (brings in OpenCV include dirs)
find_package(cv_bridge REQUIRED)

# Find TF2 packages
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find custom height mapping core library
find_package(height_mapping_core REQUIRED)

# Find PCL
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# ------------- #
# --- Build --- #
# ------------- #

# Collect all dependecies for ament_target_dependencies
set(HEIGHT_MAP_DEPENDENCIES
	rclcpp
	rclcpp_components
	sensor_msgs
	geometry_msgs
	nav_msgs
	std_msgs
	std_srvs
	grid_map_ros
	grid_map_core
	grid_map_msgs
	visualization_msgs
	tf2
	tf2_ros
	tf2_geometry_msgs
	cv_bridge
	height_mapping_core
	pcl_conversions
	pcl_ros
)

# -------------------- #
# --- Core library --- #
# -------------------- #

# Core library (templated functionality that needs explicit instantiations)
set(HEIGHT_MAPPING_CORE_LIB height_mapping_ros2_core)

add_library(${HEIGHT_MAPPING_CORE_LIB}
	src/core/height_mapper.cpp
	src/core/global_mapper.cpp
)

target_include_directories(${HEIGHT_MAPPING_CORE_LIB} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_include_directories(${HEIGHT_MAPPING_CORE_LIB} SYSTEM PRIVATE
	${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${HEIGHT_MAPPING_CORE_LIB}
	grid_map_core
	grid_map_ros
	height_mapping_core
)

target_link_libraries(${HEIGHT_MAPPING_CORE_LIB}
	${PCL_LIBRARIES}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  	"srv/SaveMap.srv"
  	DEPENDENCIES std_msgs
)

# --------------------------- #
# --- Height mapping node --- #
# --------------------------- #

# Set custom executable target name
set(HEIGHT_MAPPING_NODE_TARGET height_mapping_node)

# Add executable
add_executable(${HEIGHT_MAPPING_NODE_TARGET}
	src/ros/height_mapping_node.cpp
)

# Expose this package's headers to the target and dependents
target_include_directories(${HEIGHT_MAPPING_NODE_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
	$<INSTALL_INTERFACE:include>
)

# Pull in system include dirs (avoid propagating those downstream)
target_include_directories(${HEIGHT_MAPPING_NODE_TARGET} SYSTEM PRIVATE
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

# Link the ROS dependencies
ament_target_dependencies(${HEIGHT_MAPPING_NODE_TARGET}
	${HEIGHT_MAP_DEPENDENCIES}
)

# Link the system libraries
target_link_libraries(${HEIGHT_MAPPING_NODE_TARGET}
	${HEIGHT_MAPPING_CORE_LIB}
	${PCL_LIBRARIES}
	${OpenCV_LIBRARIES}
)

# Install executable
install(TARGETS ${HEIGHT_MAPPING_NODE_TARGET}
	DESTINATION lib/${PROJECT_NAME}
)

# --------------------------- #
# --- Global mapping node --- #
# --------------------------- #

# Set custom executable target name
set(GLOBAL_MAPPING_NODE_TARGET global_mapping_node)

# Add executable
add_executable(${GLOBAL_MAPPING_NODE_TARGET}
	src/ros/global_mapping_node.cpp
)

# Expose this package's headers to the target and dependents
target_include_directories(${GLOBAL_MAPPING_NODE_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
	$<INSTALL_INTERFACE:include>
)

# Pull in system include dirs (avoid propagating those downstream)
target_include_directories(${GLOBAL_MAPPING_NODE_TARGET} SYSTEM PRIVATE
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

# Link the ROS dependencies
ament_target_dependencies(${GLOBAL_MAPPING_NODE_TARGET}
	${HEIGHT_MAP_DEPENDENCIES}
)

# Link the system libraries
target_link_libraries(${GLOBAL_MAPPING_NODE_TARGET}
	${HEIGHT_MAPPING_CORE_LIB}
	${PCL_LIBRARIES}
	${OpenCV_LIBRARIES}
)

# Install executable
install(TARGETS ${GLOBAL_MAPPING_NODE_TARGET}
	DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------- #
# --- Sensor processor node --- #
# ----------------------------- #

# Set custom executable target name
set(SENSOR_PROCESSOR_NODE_TARGET sensor_processor_node)

# Add executable
add_executable(${SENSOR_PROCESSOR_NODE_TARGET}
	src/ros/sensor_processor_node.cpp
)

# Expose this package's headers to the target and dependents
target_include_directories(${SENSOR_PROCESSOR_NODE_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
	$<INSTALL_INTERFACE:include>
)

# Pull in system include dirs (avoid propagating those downstream)
target_include_directories(${SENSOR_PROCESSOR_NODE_TARGET} SYSTEM PRIVATE
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

# Link the ROS dependencies
ament_target_dependencies(${SENSOR_PROCESSOR_NODE_TARGET}
	${HEIGHT_MAP_DEPENDENCIES}
)

# Link the system libraries
target_link_libraries(${SENSOR_PROCESSOR_NODE_TARGET}
	${HEIGHT_MAPPING_CORE_LIB}
	${PCL_LIBRARIES}
	${OpenCV_LIBRARIES}
)

# Install executable
install(TARGETS ${SENSOR_PROCESSOR_NODE_TARGET}
	DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------- #
# --- Link generated interfaces --#
# ------------------------------- #

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${HEIGHT_MAPPING_NODE_TARGET} ${cpp_typesupport_target})
target_link_libraries(${GLOBAL_MAPPING_NODE_TARGET} ${cpp_typesupport_target})
target_link_libraries(${SENSOR_PROCESSOR_NODE_TARGET} ${cpp_typesupport_target})

# --------------------- #
# --- Install rules --- #
# --------------------- #

# Install core library so downstream packages can link if needed
install(TARGETS ${HEIGHT_MAPPING_CORE_LIB}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install configuration files
install(DIRECTORY config/
	DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
)

# -------------------------------------- #
# --- Export for DownStream Packages --- #
# -------------------------------------- #

ament_export_include_directories(include)
ament_export_dependencies(${HEIGHT_MAP_DEPENDENCIES} PCL OpenCV)
ament_export_libraries(${HEIGHT_MAPPING_CORE_LIB})
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

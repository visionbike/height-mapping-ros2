cmake_minimum_required(VERSION 3.15)

if(POLICY CMP0144)
	cmake_policy(SET CMP0144 NEW)
	set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

if(POLICY CMP0167)
	cmake_policy(SET CMP0167 NEW)
	set(CMAKE_POLICY_DEFAULT_CMP0167 NEW)
endif()

project(height_mapping_core)

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

find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# ----------------------------- #
# --- Define library target --- #
# ----------------------------- #

add_library(${PROJECT_NAME} SHARED
	src/height_map.cpp
	src/height_filter_fast.cpp
	src/height_estimator_kalman.cpp
	src/height_estimator_moving_average.cpp
	src/height_estimator_stat_mean.cpp
)

# ------------------------- #
# --- Include libraries --- #
# ------------------------- #

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>			# Important for namespace
)

#  Include the system libraries
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE 
	${PCL_INCLUDE_DIRS}
)

# ---------------------- #
# --- Link libraries --- #
# ---------------------- #

# Link against ROS2 packages (handles include paths automatically)
ament_target_dependencies(${PROJECT_NAME}
	grid_map_core
)

# --------------------- #
# --- Install rules --- #
# --------------------- #

# Install the library and export the target name
install(TARGETS ${PROJECT_NAME}
	EXPORT export_${PROJECT_NAME}
	ARCHIVE DESTINATION lib
  	LIBRARY DESTINATION lib
 	RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
	DESTINATION include/${PROJECT_NAME}
)

# -------------------------------------- #
# --- Export for DownStream Packages --- #
# -------------------------------------- #

# This allows other packages to do "find_package(height_mapping_core)"
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(grid_map_core PCL)

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

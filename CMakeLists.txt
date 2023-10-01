################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(multibot_util)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(multibot_ros2_interface REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(DEPENDENCIES
    "geometry_msgs"
    "multibot_ros2_interface"
)

set(LIBRARY_NAME "multibot_util")
file(GLOB UTIL_SOURCES "src/*.cpp")

add_library(${LIBRARY_NAME} SHARED
  ${UTIL_SOURCES}
)
target_include_directories(${LIBRARY_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_targets(${LIBRARY_NAME} HAS_LIBRARY_TARGET)
ament_target_dependencies(${LIBRARY_NAME}
  ${DEPENDENCIES}
)

################################################################################
# Install
################################################################################
install(DIRECTORY include/multibot_util
  DESTINATION include
)

install(TARGETS
  ${LIBRARY_NAME}
  EXPORT ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${DEPENDENCIES})

################################################################################
# Build test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
ament_package()
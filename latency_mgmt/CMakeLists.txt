cmake_minimum_required(VERSION 3.5)
project(latency_mgmt)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Common compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-reorder -g -O3)
endif()

# Pass through any custom flags that might already be set
if(CMAKE_CXX_FLAGS)
  message(WARNING "CMAKE_CXX_FLAGS is set to ${CMAKE_CXX_FLAGS}")
  add_compile_options(${CMAKE_CXX_FLAGS})
endif()

# Optional definitions for your project
if(PICAS)
  add_compile_options(-DPICAS)
endif()
if(PICAS_THREAD_AFFINITY)
  add_compile_options(-DPICAS_THREAD_AFFINITY)
endif()
if(LATENCY_MGMT)
  add_compile_options(-DLATENCY_MGMT)
endif()

# Gather all .cpp files
file(GLOB SOURCES "src/*.cpp")

# Include your own include/ directory
include_directories(include)

# --- Add NVTX3 include path ---
include_directories("/usr/local/cuda/targets/sbsa-linux/include")

# --- Find NVToolsExt library (for linking) ---
find_library(NVTX_LIBRARY
  NAMES nvToolsExt
  HINTS
    /usr/local/cuda/targets/sbsa-linux/lib
    /usr/local/cuda/lib64
)

if(NOT NVTX_LIBRARY)
  message(FATAL_ERROR "Could not find nvToolsExt (NVTX) library. \
  Please install or provide a search path to find_library()")
else()
  message(STATUS "Found nvToolsExt library at: ${NVTX_LIBRARY}")
endif()

# Find dependencies (ROS2 / ament / etc.)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(test_interfaces REQUIRED)
find_package(trace_picas REQUIRED)

# Create the executable
add_executable(latency_mgmt ${SOURCES})
ament_target_dependencies(latency_mgmt
  rclcpp
  test_interfaces
  trace_picas
)

# Link the NVTX library to your executable
target_link_libraries(latency_mgmt ${NVTX_LIBRARY})

# Standard ROS2 installation
install(TARGETS
  latency_mgmt
  DESTINATION lib/${PROJECT_NAME}
)

# Optionally enable tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

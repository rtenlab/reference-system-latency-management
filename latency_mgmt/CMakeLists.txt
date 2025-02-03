cmake_minimum_required(VERSION 3.5)
project(latency_mgmt)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-reorder)
endif()

if(${CMAKE_CXX_FLAGS})
  message(WARNING "CMAKE_CXX_FLAGS is set to ${CMAKE_CXX_FLAGS}")
  add_compile_options(${CMAKE_CXX_FLAGS})
endif()

if(${PICAS})
  add_compile_options(-DPICAS)
endif()
if(${PICAS_THREAD_AFFINITY})
  add_compile_options(-DPICAS_THREAD_AFFINITY)
endif()
if(${LATENCY_MGMT})
  add_compile_options(-DLATENCY_MGMT)
endif()

file (GLOB SOURCES "src/*.cpp")
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(test_interfaces REQUIRED)
find_package(trace_picas REQUIRED)

add_executable(latency_mgmt ${SOURCES})
ament_target_dependencies(latency_mgmt rclcpp test_interfaces trace_picas)
#ament_target_dependencies(latency_mgmt rclcpp)

install(TARGETS
  latency_mgmt
  DESTINATION lib/${PROJECT_NAME})    

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


cmake_minimum_required(VERSION 3.5)
project(sh_common)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_SHARED_MODULE_PREFIX "")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)
include(ConfigExtras)

include_directories(include)

add_library(heartbeat_node SHARED src/heartbeat_node.cpp)
ament_target_dependencies(heartbeat_node rclcpp std_msgs)

add_executable(sh_test_heartbeat_node src/test_heartbeat_node.cpp)
target_link_libraries(sh_test_heartbeat_node heartbeat_node)
ament_target_dependencies(sh_test_heartbeat_node rclcpp std_msgs)

add_library(sh_common_constants MODULE src/sh_common_constants.cpp)
target_include_directories(sh_common_constants PRIVATE "${Python3_INCLUDE_DIRS}")
ament_target_dependencies(sh_common_constants Boost)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS heartbeat_node sh_common_constants
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    TARGETS sh_test_heartbeat_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake rclcpp std_msgs Python3)
ament_export_libraries(heartbeat_node)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_package(CONFIG_EXTRAS "cmake/ConfigExtras.cmake")

cmake_minimum_required(VERSION 3.5)
project(sh_common)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(heartbeat_node SHARED src/heartbeat_node.cpp)
ament_target_dependencies(heartbeat_node rclcpp std_msgs)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS heartbeat_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_package()

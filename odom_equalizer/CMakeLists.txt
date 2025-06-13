cmake_minimum_required(VERSION 3.5)
project(odom_equalizer)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3) 
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# La merda di gazebo:
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-math6 REQUIRED)


include_directories(
  include/odom_equalizer
)

include_directories(
                include
)

include_directories(
  ${ignition-transport11_INCLUDE_DIRS}
  ${ignition-msgs8_INCLUDE_DIRS}
  ${ignition-math6_INCLUDE_DIRS}
)

add_library(odom_equalizer src/odom_equalizer.cpp)

ament_target_dependencies(odom_equalizer
    rclcpp
    nav_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    visualization_msgs
    )

add_executable(odom_equalizer_node src/odom_equalizer_node.cpp)

target_link_libraries(odom_equalizer_node 
odom_equalizer
ignition-transport11::ignition-transport11
ignition-msgs8::ignition-msgs8
ignition-math6::ignition-math6

)

install(TARGETS
  odom_equalizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

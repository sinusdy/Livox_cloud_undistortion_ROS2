cmake_minimum_required(VERSION 3.5)
project(deskew-livox)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++20")
# Default to C++14

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies for ROS compilation
find_package(ament_cmake REQUIRED)
# Find dependencies for C++ and ROS
find_package(rclcpp REQUIRED)
# Find dependencies for ROS messages format
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# external from ROS
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


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

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${tf2_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDE_DIRS}
)


# Add executable the generate | name must be the same as the launch file
add_executable(deskew_node src/livox_dedistortion.cpp src/data_process.cpp src/gyr_int.cpp)
# Link dependencies to the executable | name must be the same as the launch file
ament_target_dependencies(deskew_node rclcpp std_msgs tf2 tf2_geometry_msgs sensor_msgs nav_msgs)


target_link_libraries(deskew_node ${PCL_LIBRARIES})

# install the executable
install(TARGETS
deskew_node
  DESTINATION lib/${PROJECT_NAME})
# install the launch executable
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)


ament_export_include_directories(include)
ament_package()

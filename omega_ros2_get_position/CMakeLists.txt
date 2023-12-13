cmake_minimum_required(VERSION 3.8)
project(omega_ros2_get_position)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#################################################################################

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add executable
add_executable(omega_ros2_get_position src/omega_ros2_get_position.cpp src/HapticDevice.cpp)

# Link libraries
target_link_libraries(omega_ros2_get_position
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.so.3
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.so.3
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.so
  ${libusb-1.0_LIBRARIES}
  Boost::thread
)

# Target include directories
target_include_directories(omega_ros2_get_position PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set target compile features
target_compile_features(omega_ros2_get_position PUBLIC c_std_99 cxx_std_17)

# Add ROS2 dependencies
ament_target_dependencies(
  omega_ros2_get_position
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
)

# Install targets
install(TARGETS omega_ros2_get_position
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include DESTINATION share/${PROJECT_NAME})
install(DIRECTORY lib DESTINATION share/${PROJECT_NAME})
install(FILES lib/libdhd.so DESTINATION lib)
install(FILES lib/libdrd.so DESTINATION lib)
install(FILES lib/libdhd.so.3 DESTINATION lib)
install(FILES lib/libdrd.so.3 DESTINATION lib)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

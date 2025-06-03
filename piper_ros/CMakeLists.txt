cmake_minimum_required(VERSION 3.8)
project(piper_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(piper_vendor REQUIRED)
find_package(piper_hfhub_vendor REQUIRED)

include_directories(include)

# piper_node
add_executable(piper_node
  src/piper_main.cpp
  src/piper_ros/piper_node.cpp
)
target_link_libraries(piper_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  ament_index_cpp::ament_index_cpp
  piper_hfhub_vendor::hfhub
  piper_vendor::piper_lib
  ${audio_common_msgs_TARGETS}
)

# Export dependencies
ament_export_dependencies(piper_vendor)

# Install
install(TARGETS
piper_node
  DESTINATION lib/${PROJECT_NAME}
)

# TEST
if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE .clang-format)
endif()

ament_package()

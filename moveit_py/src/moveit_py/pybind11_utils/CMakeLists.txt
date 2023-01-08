add_library(pybind11_utils SHARED
  src/copy_ros_msg.cpp
)

target_include_directories(pybind11_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(pybind11_utils PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(pybind11_utils
  rclcpp
  moveit_msgs
  geometry_msgs
  pybind11
)

install(DIRECTORY include/ DESTINATION include)

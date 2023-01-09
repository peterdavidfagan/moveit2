add_library(moveit_py_utils SHARED
  src/copy_ros_msg.cpp
)

target_include_directories(moveit_py_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(moveit_py_utils PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_py_utils
  rclcpp
  moveit_msgs
  geometry_msgs
  pybind11
)

install(DIRECTORY include/ DESTINATION include)

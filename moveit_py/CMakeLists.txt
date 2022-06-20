cmake_minimum_required(VERSION 3.5)
project(moveit_py)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Set the build location and install location for a CPython extension
function(configure_build_install_location _library_name)
  install(TARGETS ${_library_name}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endfunction()

pybind11_add_module(core
	src/moveit_py/core.cpp
	src/moveit_py/moveit_core/robot_state/robot_state.cpp
	src/moveit_py/moveit_core/planning_scene/planning_scene.cpp
	src/moveit_py/utils/copy_ros_msg.cpp
	)
target_link_libraries(core PRIVATE moveit_ros_planning::moveit_cpp
				   rclcpp::rclcpp
				   moveit_core::moveit_planning_scene
				   moveit_core::moveit_utils
				   moveit_core::moveit_robot_model 
				   moveit_core::moveit_robot_state)
target_include_directories(core PUBLIC src/moveit_py/utils)
configure_build_install_location(core)

pybind11_add_module(planning 
	src/moveit_py/planning.cpp
        src/moveit_py/moveit_ros/moveit_cpp/moveit_cpp.cpp	
	src/moveit_py/moveit_ros/moveit_cpp/planning_component.cpp
	src/moveit_py/moveit_ros/planning_scene_monitor/planning_scene_monitor.cpp
	src/moveit_py/utils/copy_ros_msg.cpp
	)
target_link_libraries(planning PRIVATE moveit_ros_planning::moveit_cpp
				       moveit_ros_planning::moveit_planning_scene_monitor
				       moveit_core::moveit_utils
				       rclcpp::rclcpp)
target_include_directories(planning PUBLIC src/moveit_py/utils)
configure_build_install_location(planning)

ament_package()

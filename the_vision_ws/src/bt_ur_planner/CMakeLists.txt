cmake_minimum_required(VERSION 3.8)
project(bt_ur_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(the_interfaces REQUIRED)
find_package(rosx_introspection REQUIRED)
find_package(fixedeye_calibration REQUIRED)
find_package(tf2_eigen REQUIRED)

set(THIS_PACKAGE_DEPS
    behaviortree_ros2
    std_msgs
    std_srvs
    btcpp_ros2_interfaces
    the_interfaces
    rosx_introspection
    sensor_msgs
    rclcpp
    fixedeye_calibration
    tf2_eigen
)


add_library(listen_plugin SHARED 
src/listen_tf_client_plugin.cpp
)

add_library(tranform_plugin SHARED 
src/update_tf_plugin.cpp
)

add_library(pose_plan_plugin SHARED 
src/pose_planner_plugin.cpp)

add_library(group_state_plan_plugin SHARED 
src/man_grp_set_state_plugin.cpp)

add_library(execute_traj_plugin SHARED 
src/execute_trajectory_plugin.cpp)

add_library(deserialize_utils_plugin SHARED 
src/pose_deserialize.cpp)

target_include_directories(listen_plugin PUBLIC

  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

  $<INSTALL_INTERFACE:include>)
target_compile_definitions(listen_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(listen_plugin ${THIS_PACKAGE_DEPS})

target_include_directories(tranform_plugin PUBLIC

  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

  $<INSTALL_INTERFACE:include>)
target_compile_definitions(tranform_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(tranform_plugin ${THIS_PACKAGE_DEPS})

target_include_directories(pose_plan_plugin PUBLIC

  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

  $<INSTALL_INTERFACE:include>)
target_compile_definitions(pose_plan_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(pose_plan_plugin ${THIS_PACKAGE_DEPS})

target_include_directories(group_state_plan_plugin PUBLIC

  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

  $<INSTALL_INTERFACE:include>)
target_compile_definitions(group_state_plan_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(group_state_plan_plugin ${THIS_PACKAGE_DEPS})

target_include_directories(execute_traj_plugin PUBLIC

  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

  $<INSTALL_INTERFACE:include>)
target_compile_definitions(execute_traj_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(execute_traj_plugin ${THIS_PACKAGE_DEPS})


target_include_directories(deserialize_utils_plugin PUBLIC

  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

  $<INSTALL_INTERFACE:include>)
target_compile_definitions(deserialize_utils_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(deserialize_utils_plugin ${THIS_PACKAGE_DEPS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(
  TARGETS listen_plugin
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
listen_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
  )

install(
  TARGETS tranform_plugin
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  tranform_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)
install(
  TARGETS pose_plan_plugin
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  pose_plan_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

install(
  TARGETS group_state_plan_plugin
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  group_state_plan_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

install(
  TARGETS execute_traj_plugin
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  execute_traj_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

install(
  TARGETS deserialize_utils_plugin
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  deserialize_utils_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)
  
ament_package()

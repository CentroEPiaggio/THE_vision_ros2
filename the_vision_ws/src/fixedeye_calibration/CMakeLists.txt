cmake_minimum_required(VERSION 3.8)
project(fixedeye_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(the_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  tf2_ros
  tf2_eigen
  geometry_msgs
  visualization_msgs
  sensor_msgs
  rclcpp
  Eigen3
  the_interfaces
)
add_library(${PROJECT_NAME}_nodes SHARED 
  src/pose_weighted_average.cpp
  src/pos_cov_listener.cpp)

add_executable(listener_node src/pose_listener_node.cpp)
ament_target_dependencies(listener_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME}_nodes PUBLIC

  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_nodes ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(listener_node ${PROJECT_NAME}_nodes)

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
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_nodes
  EXPORT export_${PROJECT_NAME}_nodes
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_nodes
)

install(
  TARGETS listener_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

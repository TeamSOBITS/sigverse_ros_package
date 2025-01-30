cmake_minimum_required(VERSION 3.8)
project(sigverse_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  /usr/local/include
  /usr/local/include/bsoncxx/v_noabi
)



#-- for Launch
install(DIRECTORY launch/
DESTINATION share/${PROJECT_NAME}/launch)
#--- end Launch

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

include_directories(include)
link_directories(/usr/local/lib)
add_executable(sigverse_ros_bridge src/sigverse_ros_bridge.cpp)
ament_target_dependencies(sigverse_ros_bridge
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS
sigverse_ros_bridge
  DESTINATION lib/${PROJECT_NAME}
)
#--- end C++

target_link_libraries(sigverse_ros_bridge mongocxx bsoncxx)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nuturtle_description REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs)
find_package(nav_msgs)
find_package(turtlelib REQUIRED)
find_package(Armadillo REQUIRED) 
find_package(nusim REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

add_executable(nuslam_node src/slam.cpp)


ament_target_dependencies(nuslam_node
turtlelib
nusim
nuturtlebot_msgs
nav_msgs
sensor_msgs
rclcpp
std_msgs
std_srvs
nuturtle_description
geometry_msgs
tf2
tf2_ros
visualization_msgs)

# rosidl_target_interfaces(nuslam_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(nuslam_node "${cpp_typesupport_target}")
target_link_libraries(nuslam_node turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(nuslam_node ${ARMADILLO_LIBRARIES})
install(TARGETS
        nuslam_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        config
        srv
        DESTINATION share/${PROJECT_NAME}/
)

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

ament_package()

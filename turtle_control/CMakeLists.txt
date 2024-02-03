cmake_minimum_required(VERSION 3.8)
project(turtle_control)

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
find_package(nuturtlebot_msgs)
find_package(turtlelib)
find_package(nav_msgs)
find_package(catch_ros2)
add_executable(turtle_control_node src/turtle_control.cpp)
add_executable(turtle_odometry_node src/odometry.cpp)
add_executable(turtle_circle_node src/circle.cpp)


ament_target_dependencies(turtle_control_node 
rclcpp 
std_msgs 
std_srvs 
nuturtle_description 
geometry_msgs 
tf2 
tf2_ros 
visualization_msgs 
nuturtlebot_msgs  
turtlelib
nav_msgs)

ament_target_dependencies(turtle_odometry_node 
rclcpp 
std_msgs 
std_srvs 
nuturtle_description 
geometry_msgs 
tf2 
tf2_ros 
visualization_msgs 
nuturtlebot_msgs 
turtlelib
nav_msgs)

ament_target_dependencies(turtle_circle_node 
rclcpp 
std_msgs 
std_srvs 
nuturtle_description 
geometry_msgs 
tf2 
tf2_ros 
visualization_msgs 
nuturtlebot_msgs 
turtlelib
nav_msgs)

set(srv_files 
"srv/InitialPose.srv"
"srv/Control.srv")

rosidl_generate_interfaces(${PROJECT_NAME} 
"srv/InitialPose.srv" 
"srv/Control.srv")

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} rosidl_typesupport_cpp)


target_link_libraries(turtle_control_node "${cpp_typesupport_target}")
target_link_libraries(turtle_control_node turtlelib::turtlelib "${cpp_typesupport_target}")

rosidl_target_interfaces(turtle_odometry_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(turtle_odometry_node "${cpp_typesupport_target}")
target_link_libraries(turtle_odometry_node turtlelib::turtlelib "${cpp_typesupport_target}")

rosidl_target_interfaces(turtle_circle_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(turtle_circle_node "${cpp_typesupport_target}")
target_link_libraries(turtle_circle_node turtlelib::turtlelib "${cpp_typesupport_target}")

install(TARGETS
        turtle_control_node
        turtle_odometry_node
        turtle_circle_node
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


if(BUILD_TESTING)
  include(CTest)

  ###########################
  ## INTEGRATION TEST NODE ##
  ###########################
  # This is the node in which integration tests occur
  add_executable(turtle_control_test_node
    test/turtle_control_test.cpp
  )
  # The link libraries call links this node with catch_ros2::catch_ros2_with_node_main
  # to get the default integration test node main function
  target_link_libraries(turtle_control_test_node
    catch_ros2::catch_ros2_with_node_main
  )
  target_link_libraries(turtle_control_test_node turtlelib::turtlelib "${cpp_typesupport_target}")
  ament_target_dependencies(turtle_control_test_node
    rclcpp 
    std_msgs 
    std_srvs 
    nuturtle_description 
    geometry_msgs 
    tf2 
    tf2_ros 
    visualization_msgs 
    nuturtlebot_msgs 
    turtlelib
    nav_msgs
  )
  install(TARGETS
    turtle_control_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  #################
  ## LAUNCH FILE ##
  #################
  # This is the launch file that will be used to run the integration test
  install(FILES
    test/turtle_control_test.launch.xml  # example in XML
    DESTINATION share/${PROJECT_NAME}
  )

  ######################
  ## INTEGRATION TEST ##
  ######################
  # This CMake function allows the integration test to be run
  # when running "colcon test".
  catch_ros2_add_integration_test(TurtleControl_TestXML
    LAUNCH_FILE turtle_control_test.launch.xml  # example in XML
  )

endif()









ament_export_dependencies(rosidl_default_runtime)

ament_package()

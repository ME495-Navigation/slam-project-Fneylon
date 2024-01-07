from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.substitutions import Command, PathJoinSubstitution
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz",
                              default_value="true",
                              description="setting the rviz condition"),

        DeclareLaunchArgument("use_jsp",
                              default_value="true",
                              description="setting the jsp condition"),

        DeclareLaunchArgument("color",
                              default_value="purple",
                              description="setting the color of the base_link of the robot",
                              choices=["purple", "red", "green", "blue"]),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            namespace=LaunchConfiguration('color'),
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('use_jsp'), "' == \'true\' "]))
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            # namespace="blue",
            namespace=LaunchConfiguration('color'),
            parameters=[
                {"robot_description":
                 Command([TextSubstitution(text="xacro "),
                          PathJoinSubstitution(
                              [FindPackageShare("nuturtle_description"), "urdf/turtlebot3_burger.urdf.xacro"]), ' color:=', LaunchConfiguration('color')])}  # [1]: Used in reference for syntax

            ]

        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=LaunchConfiguration('color'),
            # namespace=LaunchConfiguration('color'),
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('use_rviz'), "' == \'true\' "])),
            arguments=[
                "-d", PathJoinSubstitution(
                    [FindPackageShare("nuturtle_description"), "config/basic_purple.rviz"])],
            on_exit=Shutdown())


    ])

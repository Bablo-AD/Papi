# Author: Addison Sears-Collins
# Date: April 29, 2024
# Description: Launch a robotic arm in Gazebo 
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
import xacro
import launch
from moveit_configs_utils import MoveItConfigsBuilder
 
def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="kai", package_name="kai_moveit"
        )
        .robot_description(file_path="config/kai.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
      
        .to_moveit_configs()
    )
    package_name_gazebo = 'kai_moveit'
    default_robot_name = 'kai'
    rviz_config_file_path = 'config/moveit.rviz'
    urdf_file_path = '/home/prasannan-robot/Desktop/Papi/ros2_ws/src/kai/src/description/robot.urdf.xacro'
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    default_rviz_config_path = os.path.join(pkg_share_gazebo, rviz_config_file_path) 
    run_move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[moveit_config.to_dict()],)
    
    rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",arguments=["-d", default_rviz_config_path], 
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
    ],)
    
    static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="static_transform_publisher",
    output="log",
    arguments=["--frame-id", "world", "--child-frame-id", "base_link"],)
    robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="both",
    parameters=[moveit_config.robot_description],)
    start_arm_controller_cmd = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'kai_controller'],
        output='screen',)
    # joint_state_broadcaster_spawner = Node(
    # package="controller_manager",
    # executable="spawner",
    # arguments=[
    #     "joint_state_broadcaster",
    #     "--controller-manager",
    #     "/controller_manager",
    # ],)
    joint_state_broadcaster_spawner = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        'joint_state_broadcaster'],
        output='screen')
    load_arm_controller_cmd = RegisterEventHandler(
    event_handler=OnProcessExit(
    target_action=joint_state_broadcaster_spawner,
    on_exit=[start_arm_controller_cmd],))
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("kai"),
            "config",
            "kai_controller.yaml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
         remappings=[
             ("~/robot_description", "/robot_description"),
        #     (
        #         "/forward_position_controller/commands",
        #         "/position_commands",
        #     ),
         ],
        output="both",
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),
            static_tf,
            robot_state_publisher,
            load_arm_controller_cmd,
            delay_rviz_after_joint_state_broadcaster_spawner,
            control_node,joint_state_broadcaster_spawner,
            joint_state_publisher_node,
            run_move_group_node,
rviz_node            
            
        ]
    )


# def generate_launch_description():
 
#   # Constants for paths to different files and folders
#   package_name_gazebo = 'kai'
 
#   default_robot_name = 'kai'
#   #gazebo_launch_file_path = 'launch'
#   #gazebo_models_path = 'models'
#   rviz_config_file_path = 'rviz/urdf_config.rviz'
#   urdf_file_path = 'src/description/robot.urdf.xacro'
#   #world_file_path = 'worlds/empty.world' # e.g. 'world/empty.world', 'world/house.world'
 
#   # Set the path to different files and folders.  
#   pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
 
#   default_rviz_config_path = os.path.join(pkg_share_gazebo, rviz_config_file_path)  
#   default_urdf_model_path = os.path.join(pkg_share_gazebo, urdf_file_path)

#   # Launch configuration variables specific to simulation
#   robot_name = LaunchConfiguration('robot_name')
#   rviz_config_file = LaunchConfiguration('rviz_config_file')
#   use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
#   use_rviz = LaunchConfiguration('use_rviz')
#   use_sim_time = LaunchConfiguration('use_sim_time')
#   #world = LaunchConfiguration('world')
   
#   # Declare the launch arguments  
#   declare_robot_name_cmd = DeclareLaunchArgument(
#     name='robot_name',
#     default_value=default_robot_name,
#     description='The name for the robot')
 
#   declare_rviz_config_file_cmd = DeclareLaunchArgument(
#     name='rviz_config_file',
#     default_value=default_rviz_config_path,
#     description='Full path to the RVIZ config file to use')
 
#   declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
#     name='use_robot_state_pub',
#     default_value='True',
#     description='Whether to start the robot state publisher')
 
#   declare_use_rviz_cmd = DeclareLaunchArgument(
#     name='use_rviz',
#     default_value='True',
#     description='Whether to start RVIZ')
  
#   # Start arm controller
#   start_arm_controller_cmd = ExecuteProcess(
#     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#         'kai_controller'],
#         output='screen')
 
   
#   # Launch joint state broadcaster
#   start_joint_state_broadcaster_cmd = ExecuteProcess(
#     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#         'joint_state_broadcaster'],
#         output='screen')
     
#   # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
#   doc = xacro.parse(open(default_urdf_model_path))
#   xacro.process_doc(doc)
#   start_robot_state_publisher_cmd = Node(
#     condition=IfCondition(use_robot_state_pub),
#     package='robot_state_publisher',
#     executable='robot_state_publisher',
#     name='robot_state_publisher',
#     output='screen',
#     parameters=[{
#       'robot_description': doc.toxml()}])
 
#   # Launch RViz
#   start_rviz_cmd = Node(
#     condition=IfCondition(use_rviz),
#     package='rviz2',
#     executable='rviz2',
#     name='rviz2',
#     output='screen',
#     arguments=['-d', rviz_config_file])  
#   robot_controllers = PathJoinSubstitution(
#         [
#             FindPackageShare("kai"),
#             "config",
#             "kai_controller.yaml",
#         ]
#     )
#   control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[robot_controllers],
#          remappings=[
#              ("~/robot_description", "/robot_description"),
#         #     (
#         #         "/forward_position_controller/commands",
#         #         "/position_commands",
#         #     ),
#          ],
#         output="both",
#     )

#   # Launch the arm controller after launching the joint state broadcaster
#   load_arm_controller_cmd = RegisterEventHandler(
#     event_handler=OnProcessExit(
#     target_action=start_joint_state_broadcaster_cmd,
#     on_exit=[start_arm_controller_cmd],))
#   delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
#         event_handler=OnProcessExit(
#             target_action=start_joint_state_broadcaster_cmd,
#             on_exit=[start_rviz_cmd],
#         )
#     )

 
  
#   # Create the launch description and populate
#   ld = LaunchDescription()
 
#   # Declare the launch options
#   ld.add_action(declare_robot_name_cmd)
#   ld.add_action(declare_rviz_config_file_cmd)
#   ld.add_action(declare_use_robot_state_pub_cmd)  
#   ld.add_action(declare_use_rviz_cmd) 
 
 
#   # Add any actions
#   ld.add_action(start_robot_state_publisher_cmd)
#   ld.add_action(start_joint_state_broadcaster_cmd)
#   ld.add_action(control_node)  
 
#   ld.add_action(load_arm_controller_cmd) 
#   ld.add_action(delay_rviz_after_joint_state_broadcaster_spawner)
  
  
 
#   return ld

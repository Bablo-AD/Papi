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
    world_file_path = 'world/empty.world'
    gazebo_launch_file_path = 'launch'
    gazebo_models_path = 'models'
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim') 
    pkg_share_gazebo = FindPackageShare(package=default_robot_name).find(default_robot_name)
    default_rviz_config_path = os.path.join(pkg_share_gazebo, rviz_config_file_path) 
    gazebo_launch_file_path = os.path.join(pkg_share_gazebo, gazebo_launch_file_path)
    gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)
    world_path = os.path.join(pkg_share_gazebo, world_file_path)
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    run_move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[moveit_config.to_dict()],)
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("kai"),
            "config",
            "kai_controller.yaml",
        ]
    )
     # Set the default pose
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    declare_robot_name_cmd = DeclareLaunchArgument(
    name='robot_name',
    default_value=default_robot_name,
    description='The name for the robot')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_x_cmd = DeclareLaunchArgument(
        name='x',
        default_value='0.0',
        description='x component of initial position, meters')
    
    declare_y_cmd = DeclareLaunchArgument(
        name='y',
        default_value='0.0',
        description='y component of initial position, meters')
        
    declare_z_cmd = DeclareLaunchArgument(
        name='z',
        default_value='0.0',
        description='z component of initial position, meters')
        
    declare_roll_cmd = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='roll angle of initial orientation, radians')
    
    declare_pitch_cmd = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='pitch angle of initial orientation, radians')
    
    declare_yaw_cmd = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='yaw angle of initial orientation, radians')
        
    # Specify the actions
    
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gazebo_models_path)
    
    start_gazebo_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments=[('gz_args', [' -r -v 6 ', world])]
    )    
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
    doc = xacro.parse(open(urdf_file_path))
    xacro.process_doc(doc)
    start_gazebo_ros_spawner_cmd = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-string', doc.toxml(),
      '-name', robot_name,
      '-allow_renaming', 'true',
      '-x', x,
      '-y', y,
      '-z', z,
      '-R', roll,
      '-P', pitch,
      '-Y', yaw
      ],
    output='screen')
    load_joint_state_broadcaster_cmd = RegisterEventHandler(
     event_handler=OnProcessExit(
     target_action=start_gazebo_ros_spawner_cmd ,
     on_exit=[joint_state_broadcaster_spawner],))
    
    
    
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),
            static_tf,
            declare_robot_name_cmd,
            robot_state_publisher,
            load_arm_controller_cmd,
            control_node,joint_state_broadcaster_spawner,
            joint_state_publisher_node,
            run_move_group_node,
rviz_node,declare_use_sim_time_cmd,
declare_world_cmd,
declare_x_cmd,
declare_y_cmd,
declare_z_cmd,
declare_roll_cmd,
declare_pitch_cmd,
declare_yaw_cmd,
set_env_vars_resources,
start_gazebo_cmd,
start_gazebo_ros_spawner_cmd    ,       
            
        ]
    )


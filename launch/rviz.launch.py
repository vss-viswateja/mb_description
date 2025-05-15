import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('mb_description')
    world = LaunchConfiguration('world.wbt')
    use_rviz = LaunchConfiguration('rviz', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'world.wbt'),
        ros2_supervisor=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>',
             'set_robot_state_publisher': True},
        ],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    camera_rgb_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'camera rgb', 'camera_rgb'],
    )
    camera_depth_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'camera depth', 'camera_depth'],
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'mb.urdf')
    ros2_control_params = os.path.join(package_dir, 'config', 'ros2control.yaml')


    my_robot_driver = WebotsController(
        robot_name='rosbot',
        parameters=[
            {
                'robot_description': robot_description_path,    #reads the urdf and all the drivers and controllers mentioned in it. It acts as a source for other ros2 controllers.
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True,      #Overrides the above robot_description parameter and makes webots publish the robot states 
                
            },
            ros2_control_params,    #This acts as a reference source yaml for the controllers being imported into the robot.
            #Note that this source YAML acts as a config for the controllers. 
        ],
        remappings=[
            ('/test_diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/test_diff_drive_controller/cmd_vel', '/cmd_vel'),
            ('/rosbot/laser', '/scan'),
            ('/test_diff_drive_controller/odom','/odom'),
            ('/imu_broadcaster/imu', '/imu')


            
        ],
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['test_diff_drive_controller'] + controller_manager_timeout,
    )
    
    waiting_nodes = WaitForControllerConnection(
        target_driver=my_robot_driver,
        nodes_to_start=[diff_drive_controller,joint_state_broadcaster_spawner]
    )
    rviz_config = os.path.join(package_dir, 'config', 'rviz.rviz')

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['d', rviz_config], 
    )
    ekf_param = os.path.join(package_dir, 'config', 'ekf.yaml')

    robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_node',
    output='screen',
    parameters=[
        ekf_param,
        {'use_sim_time': True,}
        
        ]
        
    )
    
    return LaunchDescription([
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        camera_rgb_publisher,
        camera_depth_publisher,
        my_robot_driver,
        waiting_nodes,
        rviz2_node,
        robot_localization_node,
        #map_frame_publisher,
        
                
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])

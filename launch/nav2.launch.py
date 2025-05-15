from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nav2_map = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                {'yaml_filename': '/indoor_map_vslam.yaml'}
            ]
        )
    nav2_amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
                '/path/to/amcl_params.yaml'
            ]
        )
    
    nav2_lifecycle = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[{'autostart': True}]
        )
    
    

    return LaunchDescription([

        nav2_map,
        nav2_amcl,
        nav2_lifecycle,

        
    ])
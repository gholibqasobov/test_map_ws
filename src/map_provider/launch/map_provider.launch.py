from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare the `map_file` argument
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/dosymzhan/test_map_ws/src/map_provider/maps/cabinet_unity.yaml',
        description='Path to the map file'
    )

    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file')}]
    )
    
    # Lifecycle Manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['map_server']}]
    )
        

    return LaunchDescription([
        map_file_arg,
        map_server_node,
    ])

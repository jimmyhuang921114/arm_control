from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
import os

def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config').perform(context)
    ## file available check
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found: {config_file}")
    ## open config file
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    ## create nodes
    nodes = []
    for rail_name, rail_config in config.items():
        nodes.append(
            Node(
                package='tm_rail',
                executable='rail_node',
                name=f'{rail_name}',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'target': rail_name,
                    'baud_rate': rail_config.get('baud_rate', 921600),
                    'config': config_file # Pass the config file path to the node
                }]
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='./src/tm_rail/config/tm_rail_config.yaml',
            description='Path to the TM Rail configuration YAML file.'
        ),
        OpaqueFunction(function=launch_setup)
    ])

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('door_opening_server'))
    return LaunchDescription([
        DeclareLaunchArgument('image_topic',default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('provider',default_value='auto'),
        DeclareLaunchArgument('gui_host',default_value='127.0.0.1'),
        DeclareLaunchArgument('gui_port',default_value='8878'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/door_opening.launch.py')),
             launch_arguments={'image_topic':LaunchConfiguration('image_topic'),
                               'provider':LaunchConfiguration('provider')}.items()),
        Node(package='door_opening_server',executable='door_opening_gui',name='door_opening_gui',
             output='screen',parameters=[{'image_topic':LaunchConfiguration('image_topic'),
                                         'gui_host':LaunchConfiguration('gui_host'),
                                         'gui_port':ParameterValue(LaunchConfiguration('gui_port'),value_type=int)}])])

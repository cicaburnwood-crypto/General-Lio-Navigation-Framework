from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('door_opening_server'))
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('provider', default_value='auto'),
        DeclareLaunchArgument('model_path', default_value=str(share / 'models/door_epoch11.onnx')),
        Node(package='door_opening_server', executable='detection_door_opening_server',
             name='door_opening_server', output='screen', parameters=[str(share / 'config/door_opening.yaml'),
             {'image_topic':LaunchConfiguration('image_topic'),
              'provider':LaunchConfiguration('provider'),
              'model_path':LaunchConfiguration('model_path')}])])

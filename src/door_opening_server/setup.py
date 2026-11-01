from glob import glob
from setuptools import find_packages, setup

name = 'door_opening_server'
setup(
    name=name, version='1.0.0', packages=find_packages(exclude=['tests']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                ('share/' + name, ['package.xml']),
                ('share/' + name + '/launch', glob('launch/*.launch.py')),
                ('share/' + name + '/config', glob('config/*.yaml')),
                ('share/' + name + '/web', glob('web/*.html')),
                ('share/' + name + '/models', glob('models/*.onnx') + glob('models/*.json'))],
    install_requires=['setuptools'], zip_safe=False,
    maintainer='Library Logistics', maintainer_email='maintainer@example.com',
    description='On-demand RGB-only fully-open glass door inspection',
    license='Proprietary',
    entry_points={'console_scripts': [
        'detection_door_opening_server = door_opening_server.ros_node:main',
        'door_opening_gui = door_opening_server.gui:main']})

from glob import glob
from setuptools import find_packages, setup

setup(
    name='patrol_manager',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/patrol_manager']),
        ('share/patrol_manager', ['package.xml', 'README.md']),
        ('share/patrol_manager/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanwen',
    maintainer_email='hwcao@hkclr.hk',
    description='Minimal sequential patrol executive using NavigateTo.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'patrol_manager = patrol_manager.patrol_node:main',
        'door_patrol_manager = patrol_manager.door_patrol_node:main',
    ]},
)

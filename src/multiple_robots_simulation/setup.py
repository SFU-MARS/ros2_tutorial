from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multiple_robots_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), 
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models'), glob('models/*.urdf.xacro')),
        (os.path.join('share', package_name, 'models'), glob('models/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='tianyitang_2021@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "spawn_multiple_robots= multiple_robots_simulation.spawn_multiple_robots:main",
            "bvc_controller= multiple_robots_simulation.bvc_controller:main",
            "global_position_provider= multiple_robots_simulation.global_position_provider:main",
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_reactive_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),            
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'descriptions'), glob(os.path.join('descriptions', '*.[xml]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yaml]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[rviz]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eia',
    maintainer_email='andresfmc223@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wall_follower_control = amr_reactive_control.wall_follower.wall_follower_control:main",
            "wall_follower_parameters = amr_reactive_control.wall_follower.wall_follower_params:main",
            "gap_follower_control = amr_reactive_control.gap_follower.gap_follower_control:main",
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name='controlrob'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all urdf and rviz files - 'urdf' folder.
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.[ur][rv]*'))),
        # Include all parameter (yaml) files - 'config' folder.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EthanBjerke',
    maintainer_email='ebjerke@wisc.edu',
    description='Code for the robot arm package "xarmrob" used in the University of Wisconsin-Madison course ME/ECE 439',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'command_xarm=controlrob.command_xarm:main',
        'controller_handler=controlrob.controller_handler:main',
        'xarm_kinematics=controlrob.xarm_kinematics:main',
        ],
    },
)

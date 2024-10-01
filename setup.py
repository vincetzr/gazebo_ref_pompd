from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ref_pomdp_neurips23'

setup(
    name=package_name,
    packages=find_packages(),
    version='1.0.0',
    description='Python POMDP Library for ROS 2',
    install_requires=[
        'numpy',
        'scipy',
        'tqdm',
        'matplotlib',
        'pygame',  # for some tests
        'opencv-python',  # for some tests
    ],
    license="None",
    author='Edward Kim',
    author_email='edward.kim@anu.edu.au',
    keywords=['Partially Observable Markov Decision Process', 'POMDP'],
    zip_safe=False,

    # ROS 2 specific entry points for nodes or scripts
    entry_points={
        'console_scripts': [
            'ref_solver_node = ref_pomdp_neurips23.simulator.ref_solver_node:main'  
        ],
        'launch': [
            'gazebo = gazebo_ros.launch:generate_launch_description',
    	],
    },
    # Install the launch files
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)



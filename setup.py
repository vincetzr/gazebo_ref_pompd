from setuptools import setup
from glob import glob
import os

package_name = 'ref_pomdp_neurips23'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        'simulator',
        'pomdp_py','pomdp_py.algorithms','pomdp_py.framework',
        'pomdp_py.representations','pomdp_py.representations.belief',
        'pomdp_py.representations.distribution','pomdp_py.utils',
        'pomdp_py.visual',
        'problems','problems.gridworld'
    ],
    data_files=[
        # marker for ament index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # your launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),

        # ** Install Gazebo worlds **
        ('share/' + package_name + '/simulator/worlds',
         glob('simulator/worlds/**/*.world', recursive=True)),

        # ** Install robot SDF models **
        ('share/' + package_name + '/simulator/robots',
         glob('simulator/robots/**/*.sdf', recursive=True)),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'tqdm',
        'matplotlib',
        'pygame',
        'opencv-python',
    ],
    zip_safe=False,
    author='Edward Kim',
    author_email='edward.kim@anu.edu.au',
    description='Python POMDP Library for ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ref_solver_node = simulator.ref_solver_node:main',
            'testnode = simulator.testnode:main',
		
        ],
    },
)


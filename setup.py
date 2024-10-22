from setuptools import setup
import os
from glob import glob

package_name = 'ref_pomdp_neurips23'

setup(
    name=package_name,
    version='1.0.0',
    packages=['simulator', 'pomdp_py','pomdp_py.algorithms','pomdp_py.framework',
    'pomdp_py.representations','pomdp_py.representations.belief','pomdp_py.representations.distribution','pomdp_py.utils','pomdp_py.visual'],
    # Install the launch files
    data_files=[
        # Install marker file in the package index
	('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'numpy',
        'scipy',
        'tqdm',
        'matplotlib',
        'pygame',  # for some tests
        'opencv-python',  # for some tests
    ],
    zip_safe=False,
    description='Python POMDP Library for ROS 2',
    author='Edward Kim',
    author_email='edward.kim@anu.edu.au',
    keywords=['Partially Observable Markov Decision Process', 'POMDP'],
    license="None",
    # ROS 2 specific entry points for nodes or scripts
    entry_points={
        'console_scripts': [
            'ref_solver_node = simulator.ref_solver_node:main',  
    	],
    },
    
)



from setuptools import find_packages, setup

from glob import glob

import os

package_name = 'launch_pkg'

setup(

    name=package_name,

    version='0.0.1',

    packages=find_packages(exclude=['test']),

    data_files=[

        ('share/ament_index/resource_index/packages',

            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Install launch files

        (os.path.join('share', package_name, 'launch'), 

            glob('launch/*.launch.py')),

        # Install config files

        (os.path.join('share', package_name, 'config'), 

            glob('config/*.yaml')),

    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='Benjamin Matapo',

    maintainer_email='matapobenjamin28@gmail.com',

    description='Launch files for NCL Lunabotics robot',

    license='MIT',

    entry_points={

        'console_scripts': [

            # No executable nodes - only launch files

        ],

    },

)

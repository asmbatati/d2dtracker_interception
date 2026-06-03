import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'd2dtracker_interception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asmbatati',
    maintainer_email='asmalbatati@hotmail.com',
    description='Interception strategies for the D2DTracker project.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interceptor = d2dtracker_interception.interceptor_node:main',
            'guidance = d2dtracker_interception.guidance_node:main',
        ],
    },
)

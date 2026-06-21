import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'color_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jad',
    maintainer_email='jadnoueihed1@gmail.com',
    description='Track a colored object with a webcam and mirror its position with a TurtleSim turtle.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_detector = color_tracker.color_detector:main',
            'turtle_controller = color_tracker.turtle_controller:main',
        ],
    },
)

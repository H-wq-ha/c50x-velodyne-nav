import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'semantic_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='y',
    maintainer_email='maintainer@example.com',
    description='COCO laptop, bottle, and cup semantic mapping for RealSense RGB-D.',
    license='MIT',
    entry_points={'console_scripts': [
        'semantic_mapping = semantic_mapping.node:main',
        'one_click_health_monitor = semantic_mapping.health_monitor:main',
        'semantic_navigator = semantic_mapping.semantic_navigation:main',
    ]},
)

from setuptools import find_packages, setup

package_name = 'car_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzu',
    maintainer_email='zzu@todo.com',
    description='WHEELTEC C50X chassis ROS2 driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_node = car_bringup.chassis_node:main',
            'keyboard_teleop = car_bringup.keyboard_teleop:main',
        ],
    },
)

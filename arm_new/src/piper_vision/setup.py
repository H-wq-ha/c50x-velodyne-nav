from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'piper_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='y',
    maintainer_email='y@todo.todo',
    description='YOLOv8n ONNX + RealSense D435i 3D object detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_depth_node = piper_vision.yolo_depth_node:main',
            'vision_tf_plot = piper_vision.vision_tf_plot:main',
        ],
    },
)

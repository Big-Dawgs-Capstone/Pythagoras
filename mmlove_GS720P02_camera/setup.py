from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mmlove_GS720P02_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lebron',
    maintainer_email='lebron@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fisheye_camera_publisher = mmlove_GS720P02_camera.fisheye_camera_publisher:main',
            'fisheye_camera_view = mmlove_GS720P02_camera.fisheye_camera_view:main'
        ],
    },
)

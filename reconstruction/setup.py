from setuptools import find_packages, setup

package_name = 'reconstruction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'point_cloud_node = reconstruction.point_cloud_node:main',
            'point_cloud_aggregator=reconstruction.point_cloud_aggregator:main'
        ],
    },
)

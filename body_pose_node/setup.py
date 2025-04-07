from setuptools import setup

package_name = 'body_pose_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/body_pose_launch.py']),],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Transform camera pose to body pose',
    license='MIT',
    entry_points={
        'console_scripts': [
            'body_pose_transformer = body_pose_node.body_pose_transformer:main',
        ],
    },
)


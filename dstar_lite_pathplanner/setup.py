from setuptools import setup

package_name = 'dstar_lite_pathplanner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autovendi',
    maintainer_email='autovendi@todo.todo',
    description='D* Lite Path Planner for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_follower = dstar_lite_pathplanner.path_follower:main',  # Entry point for path follower
            'twist_to_cmd = dstar_lite_pathplanner.twist_to_cmd:main',
        ],
    },
)


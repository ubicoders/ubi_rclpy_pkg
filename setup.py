from setuptools import setup
#=============
import glob
import os
#=============


package_name = 'ubi_rclpy_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        #=============
        ('share/' + package_name, glob.glob(os.path.join('launch', '*.launch.py'))),
        #=============
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hellopy_publisher = ubi_rclpy_pkg.hellopy_publisher:main',
            'hellopy_subscriber = ubi_rclpy_pkg.hellopy_subscriber:main',
            'calc_optimal_path = ubi_rclpy_pkg.interface_srv_server:main',
            'calc_optimal_path_client = ubi_rclpy_pkg.interface_srv_client:main',
            'takeoff_action_server = ubi_rclpy_pkg.interface_action_server:main',
            'takeoff_action_client = ubi_rclpy_pkg.interface_action_client:main',
        ],
    },
)

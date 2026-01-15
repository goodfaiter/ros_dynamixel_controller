from setuptools import setup

package_name = 'ros_dynamixel_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valentin Yuryev',
    maintainer_email='valentin.yuryev@gmail.com',
    description='Dynamiexl controller package.',
    license='BSD3',
    entry_points={
        'console_scripts': [
                'ros_dynamixel_controller = ros_dynamixel_controller.ros_dynamixel_controller:main',
        ],
    },
)

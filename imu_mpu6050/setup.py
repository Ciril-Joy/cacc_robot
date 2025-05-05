from setuptools import find_packages, setup

package_name = 'imu_mpu6050'

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
    maintainer='Ciril',
    maintainer_email='pi@todo.todo',
    description='ROS 2 Humble node to publish IMU data from MPU6050 on Raspberry Pi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'imu_node = imu_mpu6050.imu_node:main'
        ],
    },
)

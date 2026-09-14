from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'base'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='ubuntu',
    maintainer_email='kyb021831@gmail.com',
    description='Unit 2 hardware adapters: motors, camera, marker vision, IMU, UWB and solenoid.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'motor = base.motor:main',
        'camera = base.camera:main',
        'marker_vision = base.marker_vision:main',
        'imu = base.imu:main',
        'uwb = base.uwb:main',
        'solenoid = base.solenoid:main',
        'calibrate_camera = base.calibrate_camera:main',
        'imu_aruco_360_test = base.imu_aruco_360_test:main',
    ]},
)

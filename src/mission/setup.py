from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'mission'
setup(
    name=package_name, version='1.0.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ubuntu', maintainer_email='kyb021831@gmail.com',
    description='Unit 2 following, cargo and automatic coupling missions.', license='Apache-2.0',
    entry_points={'console_scripts': [
        'follow_leader = mission.winter.follow_leader:main',
        'coupling = mission.common.coupling:main',
        'coupled_drive = mission.common.coupled_drive:main',
        'cargo_load = mission.summer.cargo_load:main',
    ]},
)

from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'fleet'
setup(
    name=package_name, version='1.0.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ubuntu', maintainer_email='kyb021831@gmail.com',
    description='Fleet Protocol v1.0 follower endpoint for Unit 2.', license='Apache-2.0',
    entry_points={'console_scripts': [
        'follower = fleet.follower:main',
        'fake_leader = fleet.fake_leader:main',
        'rc_bridge = fleet.rc_bridge:main',
    ]},
)

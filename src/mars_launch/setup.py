from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'mars_launch'
setup(
    name=package_name, version='1.0.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ubuntu', maintainer_email='kyb021831@gmail.com',
    description='Unit 2 production and test launch composition.', license='Apache-2.0',
)

from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'drive'
setup(
    name=package_name, version='1.0.0', packages=find_packages(exclude=['test']),
    package_data={'drive': ['templates/*.html']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'flask'], zip_safe=False,
    maintainer='ubuntu', maintainer_email='kyb021831@gmail.com',
    description='Unit 2 drive arbiter, manual test input and diagnostic console.', license='Apache-2.0',
    entry_points={'console_scripts': [
        'arbiter = drive.arbiter:main',
        'manual = drive.manual:main',
        'test_console = drive.test_console:main',
    ]},
)

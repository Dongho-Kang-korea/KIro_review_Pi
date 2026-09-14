from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mars_console'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    package_data={package_name: ['templates/*.html']},
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyb',
    maintainer_email='kyb021831@gmail.com',
    description='MARS 운용 콘솔 (호기 선택 + 호기별 화면)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'console = mars_console.console:main',
        ],
    },
)

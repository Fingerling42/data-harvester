from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'data_harvester_chronicler'

setup(
    name=package_name,
    version='0.0.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan Berman',
    maintainer_email='fingerling42@proton.me',
    description='The package for recording all harvested data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_harvester_chronicler = data_harvester_chronicler.data_harvester_chronicler:main',
            'data_harvester_robonomics = data_harvester_chronicler.data_harvester_robonomics:main',
        ],
    },
)

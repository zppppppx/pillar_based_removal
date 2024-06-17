from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pillar_based_removal_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zppppppx',
    maintainer_email='zppppppx@gmail.com',
    description='Pillar-based-removal',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pillar_based_removal_node = pillar_based_removal_py.pillar_based_removal_node:main'
        ],
    },
)

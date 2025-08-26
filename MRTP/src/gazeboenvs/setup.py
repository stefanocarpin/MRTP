import os
from glob import glob
from setuptools import setup

package_name = 'gazeboenvs'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name,'params','indoor'), glob('params/indoor/*.yaml')),
        (os.path.join('share', package_name,'params','outdoor'), glob('params/outdoor/*.yaml')),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stefano Carpin',
    maintainer_email='scarpin@ucmerced.edu',
    description='Package with Gazebo environments',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

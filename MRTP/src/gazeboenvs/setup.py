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
        (os.path.join('share', package_name,'worlds/'), glob('./worlds/*')),
        (os.path.join('share', package_name,'models/pioneer2dx/'), glob('./models/pioneer2dx/*')),
        (os.path.join('share', package_name,'models/pioneer3at/'), glob('./models/pioneer3at/*')),
        (os.path.join('share', package_name,'models/husky/'), glob('./models/husky/*')),
        (os.path.join('share', package_name,'models/sick_lms111/'), glob('./models/sick_lms111/model*')),
        (os.path.join('share', package_name,'models/sick_lms111/meshes'), glob('./models/sick_lms111/meshes/*')),
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

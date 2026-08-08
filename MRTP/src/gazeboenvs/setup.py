import os
from glob import glob
from setuptools import setup

package_name = 'gazeboenvs'


def data_files_from_dir(src_dir):
    """(dest, [files]) tuple per directory level under src_dir, recursively.

    Needed for asset trees with arbitrary nesting and mixed file types
    (worlds/models/meshes/config) that a single glob() can't cover.
    """
    entries = []
    for root, _dirs, files in os.walk(src_dir):
        if not files:
            continue
        dest = os.path.join('share', package_name, root)
        entries.append((dest, [os.path.join(root, f) for f in files]))
    return entries


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
        (os.path.join('share', package_name,'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name,'config'), glob('config/*.yaml')),
        (os.path.join('lib', package_name), ['scripts/generate_orchard_world.py']),
        ] + data_files_from_dir('models') + data_files_from_dir('meshes'),
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

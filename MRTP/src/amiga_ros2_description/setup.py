import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amiga_ros2_description"


# Add files recursively
def recursive_files(directory):
    paths = []
    for root, _, files in os.walk(directory):
        if files:
            install_path = os.path.join("share", package_name, root)
            file_paths = [os.path.join(root, f) for f in files]
            paths.append((install_path, file_paths))
    return paths


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*")),
        ),
        *recursive_files("urdf"),
        *recursive_files("meshes"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)

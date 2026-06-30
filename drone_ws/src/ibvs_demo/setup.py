from setuptools import setup
from glob import glob

package_name = "ibvs_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=[],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
        (
            "share/" + package_name + "/config",
            glob("config/*"),
        ),
        (
            "share/" + package_name + "/tmux",
            glob("tmux/*"),
        ),
        (
            "share/" + package_name + "/scripts",
            glob("scripts/*"),
        ),
        (
            "share/" + package_name + "/rviz",
            glob("rviz/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aryan Singh",
    maintainer_email="aryan7109singh@gmail.com",
    description="IBVS Demo Package",
    license="BSD-3-Clause",
)
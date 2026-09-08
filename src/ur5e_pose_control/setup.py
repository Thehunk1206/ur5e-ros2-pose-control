from glob import glob
from setuptools import setup

package_name = "ur5e_pose_control"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "move_to_pose = ur5e_pose_control.move_to_pose:main",
    ]},
)

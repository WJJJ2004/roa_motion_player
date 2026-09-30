import os
from glob import glob
from setuptools import find_packages, setup


package_name = "roa_motion_player"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "motions"), glob("motions/*.yaml")),
        (os.path.join("share", package_name, "description", "urdf"),
         glob("description/urdf/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROA",
    maintainer_email="todo@todo.com",
    description="Gamepad-triggered ROA arm motion playback through RobStride commands",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "motion_player = roa_motion_player.motion_player:main",
        "hw_cmd_to_joint_state = roa_motion_player.hw_cmd_to_joint_state:main",
    ]},
)

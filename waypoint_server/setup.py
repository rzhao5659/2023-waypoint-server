import os
from glob import glob
from setuptools import setup

package_name = "waypoint_server"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Northeastern University Mars Rover Team",
    maintainer_email="northeasternmarsrover@gmail.com",
    description=(
        "Waypoint server stores waypoints and provides services to interact with them."
        "It also publishes the current waypoint that the rover must navigate to autonomously."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "server = waypoint_server.server:main",
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob

package_name = "quadrapet_feelings"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/resources", glob("resources/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi",
    maintainer_email="nathankau@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ear_control = quadrapet_feelings.ear_control:main",
            "face_control = quadrapet_feelings.face_control:main",
            "face_control_gui = quadrapet_feelings.face_control_gui:main",
            "robot_htop = quadrapet_feelings.robot_htop:main",
        ],
    },
)

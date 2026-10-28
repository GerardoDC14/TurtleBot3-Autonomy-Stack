from setuptools import setup

package_name = "motor_position_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models", ["../../models/parallel.urdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gerardo Escobar",
    maintainer_email="123440177+GerardoDC14@users.noreply.github.com",
    description=(
        "Interactive ROS 2 command publisher for bounded two-axis "
        "custom-mechanism position references."
    ),
    license="All rights reserved",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_position_controller = "
            "motor_position_controller.motor_position_controller_node:main",
        ],
    },
)

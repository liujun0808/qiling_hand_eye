from setuptools import setup

package_name = "s4_command_tools"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/dryrun_hold_command.launch.py",
                "launch/drag_teach_controller.launch.py",
                "launch/drag_teach_bringup.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config",
            [
                "config/drag_teach_joints.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="coral",
    maintainer_email="867884187@qq.com",
    description="Safe command-side utilities for S4 upper-limb bringup.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drag_teach_controller = s4_command_tools.drag_teach_controller:main",
            "hold_command_publisher = s4_command_tools.hold_command_publisher:main",
        ],
    },
)

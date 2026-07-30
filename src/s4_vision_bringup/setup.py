from setuptools import setup

package_name = "s4_vision_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/single_camera_apriltag.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="coral",
    maintainer_email="867884187@qq.com",
    description="Single-camera RealSense and AprilTag bringup for S4 hand-eye calibration.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "apriltag_pose_node = s4_vision_bringup.apriltag_pose_node:main",
        ],
    },
)

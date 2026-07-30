from setuptools import setup

package_name = "s4_handeye_calibration"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/sample_recorder.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="coral",
    maintainer_email="867884187@qq.com",
    description="Drag-teach sampling and hand-eye calibration tools for S4 upper limbs.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sample_recorder = s4_handeye_calibration.sample_recorder_node:main",
            "interactive_sample_recorder = s4_handeye_calibration.interactive_sample_recorder:main",
            "handeye_calibrate = s4_handeye_calibration.handeye_calibrate:main",
            "publish_calibration_tf = s4_handeye_calibration.publish_calibration_tf:main",
            "fk_probe = s4_handeye_calibration.fk_probe:main",
        ],
    },
)

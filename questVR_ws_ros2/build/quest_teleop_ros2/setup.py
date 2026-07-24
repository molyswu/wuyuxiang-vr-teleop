from setuptools import setup

package_name = "quest_teleop_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/urdf", ["quest_teleop_ros2/urdf/piper_description.urdf"]),
        (f"share/{package_name}/APK", ["quest_teleop_ros2/APK/teleop-debug.apk"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "oculus_pose_node = quest_teleop_ros2.oculus_pose_node:main",
            "quest_single_piper_node = quest_teleop_ros2.quest_single_piper_node:main",
            "piper_daemon = quest_teleop_ros2.piper_daemon:main",
        ],
    },
)

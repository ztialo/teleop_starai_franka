from pathlib import Path
from setuptools import setup

package_name = "leader_arm_viz"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/leader_arm_rviz.launch.py"]),
        (f"share/{package_name}/urdf", ["urdf/fr3.urdf", "urdf/cello_description.urdf"]),
        (
            f"share/{package_name}/meshes",
            [str(p) for p in Path("meshes").rglob("*") if p.is_file()],
        ),
        (
            f"share/{package_name}/rviz",
            [str(p) for p in Path("rviz").rglob("*") if p.is_file()],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zi Tao Li",
    maintainer_email="zdli@ucsc.edu",
    description="Leader arm visualization and joint state publishing.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "leader_joint_publisher = leader_arm_viz.leader_joint_publisher:main",
            "end_effector_publisher = leader_arm_viz.end_effector_publisher:main",
            # optional:
            # "read_leader = leader_arm_viz.read_leader:main",
        ],
    },
)

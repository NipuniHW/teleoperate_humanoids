from setuptools import find_packages, setup

package_name = "teleoperation_humanoid"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nipuni Wijesinghe",
    maintainer_email="hansikawijesinghe30@gmail.com",
    description=(
        "ZED2i skeleton-tracking teleoperation pipeline for humanoid robots "
        "(Pepper, NAO, Unitree G1, AgiBot X2), adaptable to new platforms."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_pepper = teleoperation_humanoid.robots.pepper:main",
            "teleop_nao = teleoperation_humanoid.robots.nao:main",
            "teleop_g1 = teleoperation_humanoid.robots.unitree_g1:main",
            "teleop_agibot_x2 = teleoperation_humanoid.robots.agibot_x2:main",
        ],
    },
)

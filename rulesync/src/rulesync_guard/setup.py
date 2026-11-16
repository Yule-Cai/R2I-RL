from setuptools import find_packages, setup


package_name = "rulesync_guard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yule",
    maintainer_email="yule@example.com",
    description="Independent snapshot-bound RuleGuard ROS 2 adapter.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ruleguard=rulesync_guard.guard_node:main",
        ],
    },
)

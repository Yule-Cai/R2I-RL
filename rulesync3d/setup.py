from setuptools import find_packages, setup


package_name = "rulesync3d"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests", "tools"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="yule",
    maintainer_email="yule@example.com",
    description="RuleSync deterministic 3-D gate and RuleGuard core.",
    license="Apache-2.0",
)

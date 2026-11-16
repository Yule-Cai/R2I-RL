from setuptools import find_packages, setup
package_name = "rulesync_nav2_bridge"
setup(name=package_name, version="0.1.0",
      packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/rulesync_nav2_bridge"]),
                  ("share/rulesync_nav2_bridge", ["package.xml"])],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="yule", maintainer_email="yule@example.com",
      description="rulesync", license="Apache-2.0",
      entry_points={"console_scripts": ["bridge=rulesync_nav2_bridge.bridge_node:main"]})

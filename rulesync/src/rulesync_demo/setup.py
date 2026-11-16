from setuptools import find_packages, setup
package_name = "rulesync_demo"
setup(name=package_name, version="0.1.0",
      packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/rulesync_demo"]),
                  ("share/rulesync_demo", ["package.xml"]),
                  ("share/rulesync_demo/launch", ["launch/rulesync_sim.launch.py",
                                                  "launch/rulesync_real.launch.py",
                                                  "launch/rulesync_guard.launch.py",
                                                  "launch/rulesync_tb3_sim.launch.py"]),
                  ("share/rulesync_demo/scripts", ["scripts/demo_runtime_rules.sh"])],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="yule", maintainer_email="yule@example.com",
      description="rulesync", license="Apache-2.0",
      entry_points={"console_scripts": ["demo_client=rulesync_demo.demo_client:main"]})

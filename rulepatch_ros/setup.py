from setuptools import find_packages, setup

package_name = 'rulepatch_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='R2I-RL',
    maintainer_email='r2i@example.com',
    description='RulePatchNet M2 ROS2 nodes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'inference = rulepatch_ros.inference_node:main',
            'auto_repair = rulepatch_ros.auto_repair:main',
            'pp_driver = rulepatch_ros.pp_driver:main',
        ],
    },
)

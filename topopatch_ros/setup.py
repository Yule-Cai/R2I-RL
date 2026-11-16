from setuptools import find_packages, setup

package_name = 'topopatch_ros'

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
    description='TopoRulePatch M3 ROS2 nodes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'topo_inference = topopatch_ros.topo_inference_node:main',
            'rule_pub = topopatch_ros.rule_pub:main',
            'gate_node = topopatch_ros.gate_node:main',
            'bench_client = topopatch_ros.bench_client:main',
            'fault_client = topopatch_ros.fault_client:main',
            'mask_delay = topopatch_ros.mask_delay:main',
            'endurance = topopatch_ros.endurance:main',
            'bench_txn = topopatch_ros.bench_txn:main',
            'topo_case_driver = topopatch_ros.topo_case_driver:main',
        ],
    },
)

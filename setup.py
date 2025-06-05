from setuptools import setup

package_name = 'can_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DEIN_NAME',
    maintainer_email='dein@email.de',
    description='CAN-Bridge für ROS2 zur Entscheidungslogik',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_bridge_node = can_interface.can_bridge_node:main',
            'controller_input_node = can_interface.controller_input_node:main',
            'hindernis_sim_node = can_interface.hindernis_sim_node:main',
            'simulate_controller_input = can_interface.simulate_controller_input:main',
        ],
    },
)


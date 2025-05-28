from setuptools import find_packages, setup

package_name = 'can_interface'

setup(
	name=package_name,
	version='0.0.0',
	packages=find_packages(exclude=['test']),
	data_files=[
    	('share/ament_index/resource_index/packages',
        	['resource/' + package_name]),
    	('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='benutzer',
	maintainer_email='thk6697@thi.de',
	description='TODO: Package description',
	license='TODO: License declaration',
	tests_require=['pytest'],
    	entry_points={
        'console_scripts': [
            'can_bridge_node = can_interface.can_bridge_node:main',
            'can_sender_node = can_interface.can_sender_node:main',
            'can_receiver_node = can_interface.can_receiver_node:main',
            'controller_input_node = can_interface.controller_input_node:main',
            'hindernis_sim_node = can_interface.hindernis_sim_node:main',
        ],
    },
)


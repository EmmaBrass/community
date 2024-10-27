from setuptools import find_packages, setup
import os
import glob

package_name = 'community'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Use glob.glob to get launch and YAML files
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'config_files'), glob.glob('config_files/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emma',
    maintainer_email='emma@brassville.com',
    description='A system of conversing LLM-driven personas.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'group_assignment_node = community.group_assignment_node:main',
            'group_node = community.group_node:main',
            'person_node = community.person_node:main',
            'relationship_manager_node = community.relationship_manager_node:main',
            'sim_pi_node = community.sim_pi_node:main',
            'sim_pi_controller_node = community.sim_pi_controller_node:main',
        ],
    },
)

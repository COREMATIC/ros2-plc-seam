from glob import glob

from setuptools import find_packages, setup

package_name = 'plc_seam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='COREMATIC Europe',
    maintainer_email='renaud.bouckenooghe@corematic.eu',
    description='Reference patterns for putting ROS 2 next to a PLC that owns motion.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'plc_bridge = plc_seam.plc_bridge_node:main',
            'plc_sim = plc_seam.plc_sim:main',
            'target_source_random = plc_seam.target_source_random:main',
            'target_source_sim = plc_seam.target_source_sim:main',
            'target_source_camera = plc_seam.target_source_camera:main',
        ],
    },
)

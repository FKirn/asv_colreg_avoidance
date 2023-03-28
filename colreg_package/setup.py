from setuptools import setup
import os
from glob import glob
package_name = 'colreg_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'parameters'), glob(os.path.join('parameters', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='filip',
    maintainer_email='filip@todo.todo',
    description='TODO: Package description',
    license='Apache License 2.0 ',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'position = colreg_package.turtle_positioning:main',
        'target_ship_control = colreg_package.target_ship_control:main',
        'main_ship_control = colreg_package.main_ship_control:main',
        'tcpa_dcpa = colreg_package.tcpa_dcpa:main',
        'avoidance_scenario = colreg_package.avoidance_scenario:main'
        ],
    },
)


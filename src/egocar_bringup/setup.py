from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'egocar_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安裝 launch 檔案
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.*py*'))),
        # 安裝 param 檔案
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
        # 安裝 rviz 檔案
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        # 注意：此處不再需要安裝 scripts，因為它們是透過 entry_points 註冊的
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Bringup package for EgoCar, handles drivers, odometry and sensor fusion.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # 主要驅動節點
            'Mcnamu_driver_X3plus = egocar_bringup.Mcnamu_driver_X3plus:main',
            'Mcnamu_driver_X3 = egocar_bringup.Mcnamu_driver_X3:main',
            'forward_node = egocar_bringup.forward_node:main',
            'calibrate_linear = egocar_bringup.calibrate_linear:main',
            'calibrate_angular = egocar_bringup.calibrate_angular:main',
           
           
        ],
    },
)

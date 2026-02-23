from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'egocar_ctrl'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='EgoCar control package with joystick and keyboard control',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'ego_joy = egocar_ctrl.ego_joy:main',
            'ego_keyboard = egocar_ctrl.ego_keyboard:main',
        ],
    },
)

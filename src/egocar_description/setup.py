import os
from glob import glob
from setuptools import setup

package_name = 'egocar_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 新增以下幾行
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf*') + glob('urdf/*.xacro*')),
        (os.path.join('share', package_name, 'urdf/sensors'), glob('urdf/sensors/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        # 添加網格檔案
        (os.path.join('share', package_name, 'meshes/X3'), glob('meshes/X3/*.STL')),
        (os.path.join('share', package_name, 'meshes/X3plus/visual'), glob('meshes/X3plus/visual/*.STL')),
        (os.path.join('share', package_name, 'meshes/X3plus/collision'), glob('meshes/X3plus/collision/*.STL')),
        (os.path.join('share', package_name, 'meshes/sensor/visual'), glob('meshes/sensor/visual/*.STL')),
        (os.path.join('share', package_name, 'meshes/sensor/collision'), glob('meshes/sensor/collision/*.STL')),
        (os.path.join('share', package_name, 'meshes/X3/mecanum'), glob('meshes/X3/mecanum/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
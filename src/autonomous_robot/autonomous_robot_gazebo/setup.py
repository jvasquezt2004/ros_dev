import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autonomous_robot_gazebo'


def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/worlds',
         glob(os.path.join('worlds', '*.world'))),
    ]

    for dirpath, _, filenames in os.walk('models'):
        if not filenames:
            continue
        install_dir = os.path.join('share', package_name, dirpath)
        file_paths = [os.path.join(dirpath, f) for f in filenames]
        data_files.append((install_dir, file_paths))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='alonso',
    maintainer_email='jorgealonsovasquez@hotmail.com',
    description='Gazebo simulation support for the robot, including worlds and launch files',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)

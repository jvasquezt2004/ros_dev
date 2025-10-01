from setuptools import find_packages, setup
from glob import glob

package_name = 'minibot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instalar archivos de launch
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Instalar archivos de description
        ('share/' + package_name + '/description', glob('description/*')),
        # Instalar archivos de rviz (si existen)
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alonso',
    maintainer_email='jorgealonsovasquez@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
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

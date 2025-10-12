from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'minibot'

def package_files(root):
    files = []
    for (path, _, filenames) in os.walk(root):
        for f in filenames:
            files.append(os.path.join(path, f))
    return files

data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/rviz',   glob('rviz/*')),
]

# Copia recursiva de description/
for f in package_files('description'):
    rel_dir = os.path.relpath(os.path.dirname(f), 'description')
    dest = os.path.join('share', package_name, 'description', rel_dir)
    data_files.append((dest, [f]))

# Copia recursiva de worlds/
for f in package_files('worlds'):
    rel_dir = os.path.relpath(os.path.dirname(f), 'worlds')
    dest = os.path.join('share', package_name, 'worlds', rel_dir)
    data_files.append((dest, [f]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alonso',
    maintainer_email='jorgealonsovasquez@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)

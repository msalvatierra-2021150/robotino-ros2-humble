from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rto_simulation'


def package_files(directory):
    paths = []

    if not os.path.isdir(directory):
        return paths

    for path, _, filenames in os.walk(directory):
        files = []
        for filename in filenames:
            files.append(os.path.join(path, filename))

        if files:
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, files))

    return paths


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),

    ('share/' + package_name,
        ['package.xml']),

    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),

    (os.path.join('share', package_name, 'worlds'),
        glob('worlds/*.sdf') + glob('worlds/*.world')),

    (os.path.join('share', package_name, 'config'),
        glob('config/*.yaml')),
]

data_files += package_files('models')
data_files += package_files('meshes')


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='salvmike0@gmail.com',
    description='Robotino Gazebo Sim simulation package',
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
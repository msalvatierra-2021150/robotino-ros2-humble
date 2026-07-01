from setuptools import find_packages, setup

package_name = 'rto_emdb_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='salvmike0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'policy_executor = robotino_emdb_bridge.policy_executor:main',
            'apriltag_to_emdb_bridge = robotino_emdb_bridge.apriltag_to_emdb_bridge:main',
        ],
    },
)
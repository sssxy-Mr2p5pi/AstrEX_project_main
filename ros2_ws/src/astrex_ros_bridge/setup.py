from setuptools import find_packages, setup

package_name = 'astrex_ros_bridge'

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
    maintainer='PercyShe',
    maintainer_email='xyshe@mail.dlut.edu.cn',
    description='AstrEX ROS 2 bridge development package.',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_cache = astrex_ros_bridge.state_cache_node:main',
            'cartpole_service = astrex_ros_bridge.cartpole_service_node:main',
        ],
    },
)

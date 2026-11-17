from setuptools import find_packages, setup

package_name = 'hover_test_mavros'

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
    maintainer='root',
    maintainer_email='a0929370607@gmail.com',
    description=(
        'Single UAV hover for ArduPilot over MAVROS: GUIDED + arm + '
        'takeoff, keyboard-driven moves in mocap coordinates, LAND. '
        'MAVROS port of hover_test.'
    ),
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hover_control = hover_test_mavros.hover_control:main',
            'hover_commander = hover_test_mavros.hover_commander:main',
        ],
    },
)

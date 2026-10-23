from setuptools import setup

package_name = 'rover_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Software Team',
    maintainer_email='rover@example.invalid',
    description='Rover ground station and onboard serial bridge',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'ground_station = rover_pkg.ground_station_node:main',
            'onboard = rover_pkg.rover_onboard_node:main',
        ],
    },
)

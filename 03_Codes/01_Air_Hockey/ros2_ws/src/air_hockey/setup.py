from setuptools import find_packages, setup

package_name = 'air_hockey'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROMER',
    maintainer_email='acartancagatay@hotmail.com',
    description='ROS Package for ROS integrated air-hockey table',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization = air_hockey.localize:main',
            'neopixel = air_hockey.neopix:main',
            'commander = air_hockey.commander:main',
        ],
    },
)

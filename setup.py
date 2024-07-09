from setuptools import find_packages, setup
from glob       import glob
import os

package_name = 'rpi_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='psu',
    maintainer_email='play4rj@aol.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = rpi_bot.keyboard_publisher:main',
            'listener = rpi_bot.keyboard_subscriber:main',
        ],
    },
)

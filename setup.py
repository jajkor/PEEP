from setuptools import find_packages, setup
from glob import glob
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
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
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
            'cmd_to_pwm_node = rpi_bot.cmd_to_pwm:main',
            'pub_hcs04_node = rpi_bot.pub_hcs04:main',
            'sub_hcs04_node = rpi_bot.sub_hcs04:main',
            'sub_sg90_node = rpi_bot.sub_sg90:main',
            #'sg90_node = rpi_bot.sg90:main',
        ],
    },
)

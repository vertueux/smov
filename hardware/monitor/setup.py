import os
from glob import glob
from setuptools import setup

package_name = 'smov_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Virtuous',
    maintainer_email='contact.vertueux@gmail.com',
    description='Driver for the LCD I2C monitor.',
    license='GNU GENERAL PUBLIC LICENSE',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example = monitor.lcd_monitor_example:main'
        ],
    },
)

import os
from glob import glob
from setuptools import setup

package_name = 'smov_lcd_panel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Virtuous',
    maintainer_email='contact.vertueux@gmail.com',
    description='Driver for the LCD I²C panel.',
    license='GNU GENERAL PUBLIC LICENSE',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = smov_lcd_panel.lcd_panel:main'
        ],
    },
)

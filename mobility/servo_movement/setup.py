import os
from glob import glob
from setuptools import setup

package_name = 'servo_movement'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Virtuous',
    maintainer_email='virtuous.open.source@gmail.com',
    description='Allow control of the 12 SMOV servos.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_movement = servo_movement.servo_control:main'
        ],
    },
)

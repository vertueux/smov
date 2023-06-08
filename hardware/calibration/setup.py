from setuptools import setup

package_name = 'calibration'

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
    description='Package used to configure your servos.',
    license='GNU GENERAL PUBLIC LICENSE',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'entry = calibration.servo_calibration:main'
        ],
    },
)

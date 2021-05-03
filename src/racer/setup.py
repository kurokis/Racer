import os
from glob import glob
from setuptools import setup

package_name = 'racer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kuroki',
    maintainer_email='kurokis@users.noreply.github.com',
    description='Racer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard = racer.keyboard:main',
            'key_ctl = racer.key_ctl:main',
            's_motor = racer.s_motor:main',
        ],
    },
)

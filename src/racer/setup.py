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
        
        # Path to the launch file
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        
        # Path to the world file
        (os.path.join('share', package_name,'worlds/'), glob('./worlds/*')),

        # Path to can models
        (os.path.join('share', package_name,'models/1can/'), glob('./models/1can/*.*')),
        (os.path.join('share', package_name,'models/3cans/'), glob('./models/3cans/*.*')),
        (os.path.join('share', package_name,'models/5cans/'), glob('./models/5cans/*.*')),
        (os.path.join('share', package_name,'models/6cans/'), glob('./models/6cans/*.*')),

        # Path to the prius sdf file
        (os.path.join('share', package_name,'models/prius/'), glob('./models/prius/*.*')),
        (os.path.join('share', package_name,'models/prius/meshes/'), glob('./models/prius/meshes/*.*')),
        (os.path.join('share', package_name,'models/prius/materials/textures'), glob('./models/prius/materials/textures/*.*')),
        
        # Path to the prius sdf file
        (os.path.join('share', package_name,'models/prius_fullscale/'), glob('./models/prius_fullscale/*.*')),
        (os.path.join('share', package_name,'models/prius_fullscale/meshes/'), glob('./models/prius_fullscale/meshes/*.*')),
        (os.path.join('share', package_name,'models/prius_fullscale/materials/textures'), glob('./models/prius_fullscale/materials/textures/*.*')),
        
        # Path to parameter json
        (os.path.join('share', package_name,'params'), glob('params/*.json')),

        # Path to the neural network model
        (os.path.join('share', package_name,'params/'), glob('params/*.pt')),
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
            'mode = racer.mode:main',
            'keyboard = racer.keyboard:main',
            'key_ctl = racer.key_ctl:main',
            'joystick = racer.joystick:main',
            'joy_ctl = racer.joy_ctl:main',
            'nn_ctl = racer.nn_ctl:main',
            'priority = racer.priority:main',
            's_motor = racer.s_motor:main',
            'r_motor = racer.r_motor:main',
            'front_camera = racer.front_camera:main',
        ],
    },
)

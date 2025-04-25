from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'puzzlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juancarlos',
    maintainer_email='jchr.2003.mx@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'puzzlebot_sim = puzzlebot_sim.puzzlebot_sim:main',
            'puzzlebot_tf = puzzlebot_sim.puzzlebot_tf:main',
            'static_tf = puzzlebot_sim.static_tf:main',
            'dyn_tf = puzzlebot_sim.dyn_tf:main',
            'tf_listener = puzzlebot_sim.tf_listener:main'
        ],
    },
)

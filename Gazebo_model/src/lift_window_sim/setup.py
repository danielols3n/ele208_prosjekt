import os
from glob import glob
from setuptools import setup

package_name = 'lift_window_sim'

setup(
    name=package_name,
    version='0.6.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'trajectories'), glob(os.path.join('trajectories', '*.csv'))),
        (os.path.join('share', package_name, 'matlab'), glob(os.path.join('matlab', '*.m'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ELE208 project group',
    maintainer_email='187519@stud.hvl.no',
    description='Teleskoplift og femleddet arm, samme DH-modell og masser som MATLAB-modellen',
    license='MIT',
    entry_points={
        'console_scripts': [
            'check_chain = lift_window_sim.check_chain:main',
        ],
    },
)

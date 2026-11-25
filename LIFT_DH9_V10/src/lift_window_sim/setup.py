# Legger robotmodellen, verdenen og banene inn i ROS 2-pakken.
from setuptools import setup
from glob import glob
package_name='lift_window_sim'
setup(name=package_name,version='0.5.0',packages=[package_name],
 data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),
             ('share/'+package_name,['package.xml'])]+
            [('share/'+package_name+'/'+folder,glob(folder+'/*')) for folder in
             ['launch','urdf','worlds','config','trajectories','matlab']],
 install_requires=['setuptools'],zip_safe=True,
 maintainer='ELE208 project group',maintainer_email='187519@stud.hvl.no',
 description='Teleskoplift og femleddet arm beskrevet med Xacro',license='MIT',
 entry_points={'console_scripts':['check_chain = lift_window_sim.check_chain:main']})

from setuptools import setup
#=============
import glob
import os
#=============

package_name = 'ubi_vrobots_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        #=============
        ('share/' + package_name, glob.glob(os.path.join('launch', '*.launch.py'))),
        #=============
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "vr_main = ubi_vrobots_ros.vr_main:main",
            "vr_ctrl_mr = ubi_vrobots_ros.vr_controller_mr:main",
        ],
    },
)

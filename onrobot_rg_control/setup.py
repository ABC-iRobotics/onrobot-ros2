import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'onrobot_rg_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer='Makány András',
    maintainer_email='andras.makany@uni-obuda.hu',
    description='Package to control an OnRobot RG gripper. Based on Takuya Kiyokawa\'s package.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'OnRobotRGSimpleControllerServer = onrobot_rg_control.OnRobotRGSimpleControllerServer:main',
                'OnRobotRGSimpleController = onrobot_rg_control.OnRobotRGSimpleController:main',
                'OnRobotRGStatusListener = onrobot_rg_control.OnRobotRGStatusListener:main',
                'OnRobotRGTcpNode = onrobot_rg_control.OnRobotRGTcpNode:main'
        ],
    },
)